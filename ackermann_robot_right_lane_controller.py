import argparse
import sys
import time
from collections import deque

try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

try:
    import RPi.GPIO as GPIO
    from gpiozero import Servo
except Exception:
    GPIO = None
    Servo = None


# Arquitetura do codigo:
# - Adaptadores de hardware: GPIO, servo, motores e camera USB.
# - Pipeline de visao: recorta a ROI, binariza as linhas brancas e mede a faixa.
# - Estrategia de controle: transforma a leitura da faixa em direcao Ackermann e velocidade.
# Essa separacao deixa mais facil trocar o controle no futuro sem mexer na camera ou nos pinos.

# ==========================
# CONFIGURACOES DO SERVO
# ==========================
SERVO_PIN = 26
SERVO_MIN_REAL = -0.50
SERVO_MAX_REAL = 1.00
SERVO_CENTRO = 0.25
SERVO_SMOOTHING = 0.30
SERVO_HISTORY = 5

# ==========================
# CONFIGURACOES DOS MOTORES DC
# Mesmos pinos do seu codigo original.
# ==========================
IN1 = 17
IN2 = 27
ENA = 18
IN3 = 22
IN4 = 23
ENB = 19
PWM_FREQ = 1000

DEFAULT_PIN_MAP = {
    "servo": SERVO_PIN,
    "motor_left_in1": IN1,
    "motor_left_in2": IN2,
    "motor_left_pwm": ENA,
    "motor_right_in1": IN3,
    "motor_right_in2": IN4,
    "motor_right_pwm": ENB,
}

VEL_BASE = 54
VEL_CURVA = 36
VEL_BUSCA = 12
VEL_MIN = 0
VEL_MAX = 80

# No seu codigo original, frente era IN1=0, IN2=1, IN3=0, IN4=1.
MOTOR_FORWARD = (0, 1)
MOTOR_BACKWARD = (1, 0)

# ==========================
# CAMERA USB / VISAO
# ==========================
CAMERA_INDEX = 0
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FPS = 30
ROI_TOP = 0.42
LINHA_BRANCA_THRESH = 185

# Alvo nominal da linha continua direita. Quando as duas bordas da faixa
# aparecem, o alvo passa a ser calculado pela largura da faixa.
RIGHT_TARGET_PX = 42
KP_OFFSET = 1.0 / 76.0
KP_SLOPE = 1.05


def limitar(value, low, high):
    return max(low, min(high, value))


def formatar_valor(value):
    return "None" if value is None else f"{value:.1f}"


class EstrategiaControleFaixaDireita:
    """Padrao Strategy: encapsula a decisao de manter o carro na faixa direita."""

    def calcular(self, faixa, ultimo_comando_direcao, direcao_busca):
        linha_direita = faixa["right_offset"]
        linha_esquerda = faixa["left_offset"]
        alvo = faixa["target"]
        inclinacao = faixa["slope"]

        if linha_direita is None:
            comando_direcao = ultimo_comando_direcao * 0.92 or direcao_busca
            velocidade = VEL_BUSCA
        else:
            comando_direcao = (linha_direita - alvo) * KP_OFFSET + inclinacao * KP_SLOPE

            # Se a linha continua esquerda apareceu perto demais, empurra para a direita.
            if linha_esquerda is not None and linha_esquerda > -50 and linha_direita > 44:
                comando_direcao += 0.34

            # Quando a tracejada some, a leitura pode estar perto da contramao; reforca a direita.
            if faixa["dash_offset"] is None and linha_direita > 78:
                comando_direcao += 0.16
            if faixa["dash_offset"] is None and linha_direita > 98:
                comando_direcao += 0.26

            # Linha direita perto demais: corrige suavemente para nao sair pela borda.
            if linha_direita < 36:
                comando_direcao -= 0.22

            velocidade = VEL_CURVA if abs(comando_direcao) > 0.38 or faixa["confidence"] < 0.55 else VEL_BASE

        return limitar(comando_direcao, -1.0, 1.0), int(limitar(velocidade, 0, VEL_MAX))


class CarroAutonomoAckermann:
    def __init__(self, args):
        self.args = args
        self.simulacao_sem_hardware = args.simulacao_sem_hardware
        self.mostrar_janelas = not args.sem_janelas
        self.rodando = True

        self.camera = None
        self.servo = None
        self.pwm_motor_esq = None
        self.pwm_motor_dir = None

        self.historico_servo = deque(maxlen=SERVO_HISTORY)
        self.servo_atual = SERVO_CENTRO
        self.ultimo_comando_direcao = 0.0
        self.ultimo_deslocamento_direita = RIGHT_TARGET_PX
        self.contador_frames = 0
        self.ultimo_log = 0
        self.estrategia_controle = EstrategiaControleFaixaDireita()

        self.configurar_gpio()
        self.configurar_servo()
        self.configurar_camera()

    def configurar_gpio(self):
        if self.simulacao_sem_hardware:
            print("DRY-RUN: GPIO desativado")
            return
        if GPIO is None:
            raise RuntimeError("RPi.GPIO nao esta disponivel. Rode na Raspberry Pi ou use --dry-run.")

        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        for pin in self.pinos_motor():
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_motor_esq = GPIO.PWM(self.args.pin_left_pwm, PWM_FREQ)
        self.pwm_motor_dir = GPIO.PWM(self.args.pin_right_pwm, PWM_FREQ)
        self.pwm_motor_esq.start(0)
        self.pwm_motor_dir.start(0)
        self.definir_direcao_motores(forward=True)
        print("GPIO configurado com sucesso em modo BCM")
        print(
            "Pinagem: "
            f"servo={self.args.pin_servo}, "
            f"motor_esq IN1/IN2/PWM={self.args.pin_left_in1}/{self.args.pin_left_in2}/{self.args.pin_left_pwm}, "
            f"motor_dir IN3/IN4/PWM={self.args.pin_right_in1}/{self.args.pin_right_in2}/{self.args.pin_right_pwm}"
        )

    def pinos_motor(self):
        return [
            self.args.pin_left_in1,
            self.args.pin_left_in2,
            self.args.pin_left_pwm,
            self.args.pin_right_in1,
            self.args.pin_right_in2,
            self.args.pin_right_pwm,
        ]

    def configurar_servo(self):
        if self.simulacao_sem_hardware:
            print("DRY-RUN: servo desativado")
            return
        if Servo is None:
            raise RuntimeError("gpiozero.Servo nao esta disponivel. Instale gpiozero ou use --dry-run.")

        self.servo = Servo(
            self.args.pin_servo,
            min_pulse_width=self.args.servo_min_pulse,
            max_pulse_width=self.args.servo_max_pulse,
        )
        self.servo.value = SERVO_CENTRO
        self.servo_atual = SERVO_CENTRO
        print(f"Servo configurado no pino BCM {self.args.pin_servo}; centro={SERVO_CENTRO}")

    def configurar_camera(self):
        if cv2 is None:
            raise RuntimeError(
                "OpenCV/numpy nao estao disponiveis. Instale com: "
                "sudo apt install python3-opencv python3-numpy"
            )
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else 0
        self.camera = cv2.VideoCapture(self.args.camera, backend)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
        self.camera.set(cv2.CAP_PROP_FPS, self.args.fps)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.camera.isOpened():
            raise RuntimeError(
                f"Nao consegui abrir a camera USB index={self.args.camera}. "
                "Teste com: ls /dev/video*"
            )

        ok, quadro = self.camera.read()
        if not ok or quadro is None:
            raise RuntimeError("Camera abriu, mas nao entregou quadro.")

        print(f"Camera USB conectada: index={self.args.camera}, quadro={quadro.shape[1]}x{quadro.shape[0]}")

    def definir_direcao_motores(self, forward=True):
        if self.simulacao_sem_hardware:
            return
        a, b = MOTOR_FORWARD if forward else MOTOR_BACKWARD
        GPIO.output(self.args.pin_left_in1, a)
        GPIO.output(self.args.pin_left_in2, b)
        GPIO.output(self.args.pin_right_in1, a)
        GPIO.output(self.args.pin_right_in2, b)

    def definir_velocidades_motores(self, vel_esq, vel_dir):
        vel_esq = int(limitar(vel_esq, VEL_MIN, VEL_MAX))
        vel_dir = int(limitar(vel_dir, VEL_MIN, VEL_MAX))
        if self.simulacao_sem_hardware:
            return
        self.pwm_motor_esq.ChangeDutyCycle(vel_esq)
        self.pwm_motor_dir.ChangeDutyCycle(vel_dir)

    def parar_motores(self):
        self.definir_velocidades_motores(0, 0)

    def direcao_para_valor_servo(self, comando_direcao):
        comando_direcao = limitar(comando_direcao, -1.0, 1.0)
        if comando_direcao >= 0:
            return SERVO_CENTRO + comando_direcao * (SERVO_MAX_REAL - SERVO_CENTRO)
        return SERVO_CENTRO + abs(comando_direcao) * (SERVO_MIN_REAL - SERVO_CENTRO)

    def aplicar_servo(self, comando_direcao):
        target = self.direcao_para_valor_servo(comando_direcao)
        self.historico_servo.append(target)
        average = sum(self.historico_servo) / len(self.historico_servo)
        self.servo_atual = self.servo_atual * (1 - SERVO_SMOOTHING) + average * SERVO_SMOOTHING
        self.servo_atual = limitar(self.servo_atual, SERVO_MIN_REAL, SERVO_MAX_REAL)

        if not self.simulacao_sem_hardware and self.servo is not None:
            self.servo.value = self.servo_atual
        return self.servo_atual

    def pre_processar_imagem(self, quadro):
        # A ROI inferior reduz distrações: placas, parede e horizonte nao entram no controle da faixa.
        roi_y = int(quadro.shape[0] * ROI_TOP)
        roi = quadro[roi_y:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mascara = cv2.threshold(blur, self.args.white_threshold, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
        return mascara, roi_y

    def varrer_grupos(self, mascara, y):
        groups = []
        current = None
        center_x = mascara.shape[1] // 2

        for x in range(0, mascara.shape[1], 2):
            offset = x - center_x
            white = mascara[y, x] > 0
            if white:
                if current is None:
                    current = {"start": offset, "end": offset, "count": 0}
                current["end"] = offset
                current["count"] += 1
            elif current is not None:
                groups.append(self.para_grupo(current))
                current = None

        if current is not None:
            groups.append(self.para_grupo(current))

        return [g for g in groups if 2 <= g["width"] <= 38]

    @staticmethod
    def para_grupo(raw):
        return {
            "start": raw["start"],
            "end": raw["end"],
            "center": (raw["start"] + raw["end"]) / 2.0,
            "width": raw["end"] - raw["start"] + 2,
            "count": raw["count"],
        }

    @staticmethod
    def deslocamento_ponderado(items):
        if not items:
            return None
        total = sum(item["weight"] for item in items)
        return sum(item["offset"] * item["weight"] for item in items) / total

    @staticmethod
    def inclinacao_linha(items):
        if len(items) < 2:
            return 0.0
        ordered = sorted(items, key=lambda item: item["dist"])
        near = ordered[:3]
        far = ordered[-3:]
        near_offset = sum(item["offset"] for item in near) / len(near)
        far_offset = sum(item["offset"] for item in far) / len(far)
        span = max(1.0, far[-1]["dist"] - near[0]["dist"])
        return limitar((far_offset - near_offset) / span, -1.0, 1.0)

    @staticmethod
    def alvo_faixa(right_offset, dash_offset):
        if right_offset is None or dash_offset is None:
            return None
        lane_width = right_offset - dash_offset
        if lane_width < 34 or lane_width > 120:
            return None
        return limitar(lane_width * 0.42, 30, 50)

    def detectar_faixa(self, quadro):
        mascara, roi_y = self.pre_processar_imagem(quadro)
        rows = [int(mascara.shape[0] * p) for p in [0.86, 0.78, 0.69, 0.60, 0.50, 0.40, 0.31, 0.23]]
        usable_right = []
        usable_left = []
        usable_dash = []
        expected_right = limitar(self.ultimo_deslocamento_direita, 28, 112)

        for i, y in enumerate(rows):
            dist = 18 + i * 22
            groups = self.varrer_grupos(mascara, y)
            if len(groups) >= 7:
                continue

            right_candidates = [g for g in groups if 14 < g["center"] < 145]
            left_candidates = [g for g in groups if g["center"] < -30]
            dash_candidates = [g for g in groups if abs(g["center"]) <= 42]

            if right_candidates:
                best = sorted(
                    right_candidates,
                    key=lambda g: abs(g["center"] - expected_right) + max(0, g["width"] - 16) * 2,
                )[0]
                usable_right.append({
                    "offset": best["center"],
                    "dist": dist,
                    "weight": limitar(2.2 - dist / 125.0, 0.45, 2.0),
                    "row": y,
                })

            if left_candidates:
                best = sorted(left_candidates, key=lambda g: abs(g["center"] + 52))[0]
                usable_left.append({
                    "offset": best["center"],
                    "dist": dist,
                    "weight": limitar(1.6 - dist / 175.0, 0.45, 1.45),
                    "row": y,
                })

            if dash_candidates:
                best = sorted(dash_candidates, key=lambda g: abs(g["center"]))[0]
                usable_dash.append({"offset": best["center"], "dist": dist, "weight": 1.0, "row": y})

        right_offset = self.deslocamento_ponderado(usable_right)
        left_offset = self.deslocamento_ponderado(usable_left)
        dash_offset = self.deslocamento_ponderado(usable_dash)
        slope = self.inclinacao_linha(usable_right)
        target = self.alvo_faixa(right_offset, dash_offset) or RIGHT_TARGET_PX
        confidence = limitar(len(usable_right) * 0.18, 0.0, 1.0)

        if right_offset is not None:
            self.ultimo_deslocamento_direita = right_offset

        return {
            "mascara": mascara,
            "roi_y": roi_y,
            "right_offset": right_offset,
            "left_offset": left_offset,
            "dash_offset": dash_offset,
            "target": target,
            "slope": slope,
            "confidence": confidence,
            "right_points": usable_right,
            "left_points": usable_left,
            "dash_points": usable_dash,
        }

    def controlador(self, faixa):
        comando_direcao, velocidade = self.estrategia_controle.calcular(
            faixa,
            self.ultimo_comando_direcao,
            self.args.search_steer,
        )
        self.ultimo_comando_direcao = comando_direcao
        return comando_direcao, velocidade

    def calcular_velocidades_motores(self, speed, comando_direcao):
        turn = abs(comando_direcao)
        inner_factor = 1.0 - min(0.18, turn * 0.12)
        if comando_direcao < -0.05:
            return int(speed * inner_factor), speed
        if comando_direcao > 0.05:
            return speed, int(speed * inner_factor)
        return speed, speed

    def desenhar_debug(self, quadro, faixa, comando_direcao, valor_servo, velocidades, fps):
        roi_y = faixa["roi_y"]
        height, width = quadro.shape[:2]
        center_x = width // 2

        cv2.rectangle(quadro, (0, roi_y), (width - 1, height - 1), (0, 180, 0), 1)
        cv2.line(quadro, (center_x, roi_y), (center_x, height), (255, 160, 0), 1)

        for item in faixa["right_points"]:
            x = int(center_x + item["offset"])
            y = int(roi_y + item["row"])
            cv2.circle(quadro, (x, y), 4, (255, 255, 0), -1)

        for item in faixa["left_points"]:
            x = int(center_x + item["offset"])
            y = int(roi_y + item["row"])
            cv2.circle(quadro, (x, y), 4, (0, 0, 255), -1)

        target_x = int(center_x + faixa["target"])
        cv2.line(quadro, (target_x, roi_y), (target_x, height), (0, 255, 255), 1)

        lines = [
            f"right={formatar_valor(faixa['right_offset'])} left={formatar_valor(faixa['left_offset'])} dash={formatar_valor(faixa['dash_offset'])}",
            f"target={faixa['target']:.1f} slope={faixa['slope']:.2f} conf={faixa['confidence']:.2f}",
            f"steer={comando_direcao:.2f} servo={valor_servo:.2f}",
            f"motor L/R={velocidades[0]}/{velocidades[1]} fps={fps:.1f}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(quadro, text, (8, 22 + i * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
            cv2.putText(quadro, text, (8, 22 + i * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 1)
        return quadro

    def registrar_status(self, faixa, comando_direcao, valor_servo, velocidades, fps):
        now = time.time()
        if now - self.ultimo_log < self.args.log_interval:
            return
        self.ultimo_log = now
        print(
            f"quadro={self.contador_frames} "
            f"right={formatar_valor(faixa['right_offset'])} left={formatar_valor(faixa['left_offset'])} "
            f"target={faixa['target']:.1f} steer={comando_direcao:.2f} servo={valor_servo:.2f} "
            f"motor={velocidades[0]}/{velocidades[1]} conf={faixa['confidence']:.2f} fps={fps:.1f}"
        )

    def executar(self):
        print("\n--- INICIANDO NAVEGACAO AUTONOMA ---")
        print("Use Ctrl+C para parar. Com janela aberta, pressione q ou ESC.")
        self.definir_direcao_motores(forward=True)

        try:
            while self.rodando:
                start = time.time()
                ok, quadro = self.camera.read()
                if not ok or quadro is None:
                    print("ERRO: falha ao capturar quadro")
                    self.parar_motores()
                    time.sleep(0.05)
                    continue

                self.contador_frames += 1
                faixa = self.detectar_faixa(quadro)
                comando_direcao, speed = self.controlador(faixa)
                valor_servo = self.aplicar_servo(comando_direcao)
                velocidades = self.calcular_velocidades_motores(speed, comando_direcao)
                self.definir_velocidades_motores(*velocidades)

                fps = 1.0 / max(time.time() - start, 0.001)
                self.registrar_status(faixa, comando_direcao, valor_servo, velocidades, fps)

                if self.mostrar_janelas:
                    debug = self.desenhar_debug(quadro.copy(), faixa, comando_direcao, valor_servo, velocidades, fps)
                    cv2.imshow("camera", debug)
                    cv2.imshow("mascara", faixa["mascara"])
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        print("Parada solicitada pela tecla")
                        break

                if self.args.loop_delay > 0:
                    time.sleep(self.args.loop_delay)

        except KeyboardInterrupt:
            print("\nInterrompido pelo usuario")
        finally:
            self.limpar_recursos()

    def limpar_recursos(self):
        print("\n--- FINALIZANDO COM SEGURANCA ---")
        self.rodando = False
        try:
            self.parar_motores()
            time.sleep(0.15)
            if self.servo is not None:
                self.servo.value = SERVO_CENTRO
                time.sleep(0.25)
            if self.pwm_motor_esq is not None:
                self.pwm_motor_esq.stop()
            if self.pwm_motor_dir is not None:
                self.pwm_motor_dir.stop()
            if GPIO is not None and not self.simulacao_sem_hardware:
                GPIO.cleanup()
            if self.camera is not None:
                self.camera.release()
            if self.mostrar_janelas:
                cv2.destroyAllWindows()
        finally:
            print("Sistema finalizado")


def ler_argumentos():
    parser = argparse.ArgumentParser(description="Carro autonomo Ackermann para Raspberry Pi + camera USB")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="Indice da camera USB, normalmente 0")
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT)
    parser.add_argument("--fps", type=int, default=CAMERA_FPS)
    parser.add_argument("--white-threshold", type=int, default=LINHA_BRANCA_THRESH)
    parser.add_argument("--search-steer", type=float, default=0.18, help="Direcao usada quando perde a linha")
    parser.add_argument("--servo-min-pulse", type=float, default=0.0005)
    parser.add_argument("--servo-max-pulse", type=float, default=0.0025)
    parser.add_argument("--pin-servo", type=int, default=DEFAULT_PIN_MAP["servo"], help="Pino BCM do servo Ackermann")
    parser.add_argument("--pin-left-in1", type=int, default=DEFAULT_PIN_MAP["motor_left_in1"], help="Pino BCM IN1 motor esquerdo")
    parser.add_argument("--pin-left-in2", type=int, default=DEFAULT_PIN_MAP["motor_left_in2"], help="Pino BCM IN2 motor esquerdo")
    parser.add_argument("--pin-left-pwm", type=int, default=DEFAULT_PIN_MAP["motor_left_pwm"], help="Pino BCM PWM/ENA motor esquerdo")
    parser.add_argument("--pin-right-in1", type=int, default=DEFAULT_PIN_MAP["motor_right_in1"], help="Pino BCM IN3 motor direito")
    parser.add_argument("--pin-right-in2", type=int, default=DEFAULT_PIN_MAP["motor_right_in2"], help="Pino BCM IN4 motor direito")
    parser.add_argument("--pin-right-pwm", type=int, default=DEFAULT_PIN_MAP["motor_right_pwm"], help="Pino BCM PWM/ENB motor direito")
    parser.add_argument("--loop-delay", type=float, default=0.0)
    parser.add_argument("--log-interval", type=float, default=0.5)
    parser.add_argument("--dry-run", dest="simulacao_sem_hardware", action="store_true", help="Nao aciona GPIO/servo; usa somente camera e debug")
    parser.add_argument("--no-display", dest="sem_janelas", action="store_true", help="Roda sem janelas OpenCV, bom para SSH")
    return parser.parse_args()


if __name__ == "__main__":
    args = ler_argumentos()
    print("--- SISTEMA AUTONOMO ACKERMANN + CAMERA USB INICIADO ---")
    car = None
    try:
        car = CarroAutonomoAckermann(args)
        time.sleep(1.0)
        car.executar()
    except Exception as error:
        print(f"ERRO FATAL: {error}")
        if car is not None:
            car.limpar_recursos()
        elif GPIO is not None and not args.simulacao_sem_hardware:
            GPIO.cleanup()
        sys.exit(1)
