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


def clamp(value, low, high):
    return max(low, min(high, value))


def fmt(value):
    return "None" if value is None else f"{value:.1f}"


class AutonomousCar:
    def __init__(self, args):
        self.args = args
        self.dry_run = args.dry_run
        self.display = not args.no_display
        self.running = True

        self.cap = None
        self.my_servo = None
        self.pwm_esq = None
        self.pwm_dir = None

        self.servo_history = deque(maxlen=SERVO_HISTORY)
        self.servo_atual = SERVO_CENTRO
        self.last_steer_cmd = 0.0
        self.last_right_offset = RIGHT_TARGET_PX
        self.frame_count = 0
        self.last_log = 0

        self.setup_gpio()
        self.setup_servo()
        self.setup_camera()

    def setup_gpio(self):
        if self.dry_run:
            print("DRY-RUN: GPIO desativado")
            return
        if GPIO is None:
            raise RuntimeError("RPi.GPIO nao esta disponivel. Rode na Raspberry Pi ou use --dry-run.")

        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        for pin in self.motor_pins():
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_esq = GPIO.PWM(self.args.pin_left_pwm, PWM_FREQ)
        self.pwm_dir = GPIO.PWM(self.args.pin_right_pwm, PWM_FREQ)
        self.pwm_esq.start(0)
        self.pwm_dir.start(0)
        self.set_motor_direction(forward=True)
        print("GPIO configurado com sucesso em modo BCM")
        print(
            "Pinagem: "
            f"servo={self.args.pin_servo}, "
            f"motor_esq IN1/IN2/PWM={self.args.pin_left_in1}/{self.args.pin_left_in2}/{self.args.pin_left_pwm}, "
            f"motor_dir IN3/IN4/PWM={self.args.pin_right_in1}/{self.args.pin_right_in2}/{self.args.pin_right_pwm}"
        )

    def motor_pins(self):
        return [
            self.args.pin_left_in1,
            self.args.pin_left_in2,
            self.args.pin_left_pwm,
            self.args.pin_right_in1,
            self.args.pin_right_in2,
            self.args.pin_right_pwm,
        ]

    def setup_servo(self):
        if self.dry_run:
            print("DRY-RUN: servo desativado")
            return
        if Servo is None:
            raise RuntimeError("gpiozero.Servo nao esta disponivel. Instale gpiozero ou use --dry-run.")

        self.my_servo = Servo(
            self.args.pin_servo,
            min_pulse_width=self.args.servo_min_pulse,
            max_pulse_width=self.args.servo_max_pulse,
        )
        self.my_servo.value = SERVO_CENTRO
        self.servo_atual = SERVO_CENTRO
        print(f"Servo configurado no pino BCM {self.args.pin_servo}; centro={SERVO_CENTRO}")

    def setup_camera(self):
        if cv2 is None:
            raise RuntimeError(
                "OpenCV/numpy nao estao disponiveis. Instale com: "
                "sudo apt install python3-opencv python3-numpy"
            )
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else 0
        self.cap = cv2.VideoCapture(self.args.camera, backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.args.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Nao consegui abrir a camera USB index={self.args.camera}. "
                "Teste com: ls /dev/video*"
            )

        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera abriu, mas nao entregou frame.")

        print(f"Camera USB conectada: index={self.args.camera}, frame={frame.shape[1]}x{frame.shape[0]}")

    def set_motor_direction(self, forward=True):
        if self.dry_run:
            return
        a, b = MOTOR_FORWARD if forward else MOTOR_BACKWARD
        GPIO.output(self.args.pin_left_in1, a)
        GPIO.output(self.args.pin_left_in2, b)
        GPIO.output(self.args.pin_right_in1, a)
        GPIO.output(self.args.pin_right_in2, b)

    def set_motor_speeds(self, vel_esq, vel_dir):
        vel_esq = int(clamp(vel_esq, VEL_MIN, VEL_MAX))
        vel_dir = int(clamp(vel_dir, VEL_MIN, VEL_MAX))
        if self.dry_run:
            return
        self.pwm_esq.ChangeDutyCycle(vel_esq)
        self.pwm_dir.ChangeDutyCycle(vel_dir)

    def stop_motors(self):
        self.set_motor_speeds(0, 0)

    def steer_to_servo_value(self, steer_cmd):
        steer_cmd = clamp(steer_cmd, -1.0, 1.0)
        if steer_cmd >= 0:
            return SERVO_CENTRO + steer_cmd * (SERVO_MAX_REAL - SERVO_CENTRO)
        return SERVO_CENTRO + abs(steer_cmd) * (SERVO_MIN_REAL - SERVO_CENTRO)

    def set_servo(self, steer_cmd):
        target = self.steer_to_servo_value(steer_cmd)
        self.servo_history.append(target)
        average = sum(self.servo_history) / len(self.servo_history)
        self.servo_atual = self.servo_atual * (1 - SERVO_SMOOTHING) + average * SERVO_SMOOTHING
        self.servo_atual = clamp(self.servo_atual, SERVO_MIN_REAL, SERVO_MAX_REAL)

        if not self.dry_run and self.my_servo is not None:
            self.my_servo.value = self.servo_atual
        return self.servo_atual

    def preprocess(self, frame):
        roi_y = int(frame.shape[0] * ROI_TOP)
        roi = frame[roi_y:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, self.args.white_threshold, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask, roi_y

    def scan_groups(self, mask, y):
        groups = []
        current = None
        center_x = mask.shape[1] // 2

        for x in range(0, mask.shape[1], 2):
            offset = x - center_x
            white = mask[y, x] > 0
            if white:
                if current is None:
                    current = {"start": offset, "end": offset, "count": 0}
                current["end"] = offset
                current["count"] += 1
            elif current is not None:
                groups.append(self.to_group(current))
                current = None

        if current is not None:
            groups.append(self.to_group(current))

        return [g for g in groups if 2 <= g["width"] <= 38]

    @staticmethod
    def to_group(raw):
        return {
            "start": raw["start"],
            "end": raw["end"],
            "center": (raw["start"] + raw["end"]) / 2.0,
            "width": raw["end"] - raw["start"] + 2,
            "count": raw["count"],
        }

    @staticmethod
    def weighted_offset(items):
        if not items:
            return None
        total = sum(item["weight"] for item in items)
        return sum(item["offset"] * item["weight"] for item in items) / total

    @staticmethod
    def line_slope(items):
        if len(items) < 2:
            return 0.0
        ordered = sorted(items, key=lambda item: item["dist"])
        near = ordered[:3]
        far = ordered[-3:]
        near_offset = sum(item["offset"] for item in near) / len(near)
        far_offset = sum(item["offset"] for item in far) / len(far)
        span = max(1.0, far[-1]["dist"] - near[0]["dist"])
        return clamp((far_offset - near_offset) / span, -1.0, 1.0)

    @staticmethod
    def lane_target(right_offset, dash_offset):
        if right_offset is None or dash_offset is None:
            return None
        lane_width = right_offset - dash_offset
        if lane_width < 34 or lane_width > 120:
            return None
        return clamp(lane_width * 0.42, 30, 50)

    def detect_lane(self, frame):
        mask, roi_y = self.preprocess(frame)
        rows = [int(mask.shape[0] * p) for p in [0.86, 0.78, 0.69, 0.60, 0.50, 0.40, 0.31, 0.23]]
        usable_right = []
        usable_left = []
        usable_dash = []
        expected_right = clamp(self.last_right_offset, 28, 112)

        for i, y in enumerate(rows):
            dist = 18 + i * 22
            groups = self.scan_groups(mask, y)
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
                    "weight": clamp(2.2 - dist / 125.0, 0.45, 2.0),
                    "row": y,
                })

            if left_candidates:
                best = sorted(left_candidates, key=lambda g: abs(g["center"] + 52))[0]
                usable_left.append({
                    "offset": best["center"],
                    "dist": dist,
                    "weight": clamp(1.6 - dist / 175.0, 0.45, 1.45),
                    "row": y,
                })

            if dash_candidates:
                best = sorted(dash_candidates, key=lambda g: abs(g["center"]))[0]
                usable_dash.append({"offset": best["center"], "dist": dist, "weight": 1.0, "row": y})

        right_offset = self.weighted_offset(usable_right)
        left_offset = self.weighted_offset(usable_left)
        dash_offset = self.weighted_offset(usable_dash)
        slope = self.line_slope(usable_right)
        target = self.lane_target(right_offset, dash_offset) or RIGHT_TARGET_PX
        confidence = clamp(len(usable_right) * 0.18, 0.0, 1.0)

        if right_offset is not None:
            self.last_right_offset = right_offset

        return {
            "mask": mask,
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

    def controller(self, lane):
        right = lane["right_offset"]
        left = lane["left_offset"]
        target = lane["target"]
        slope = lane["slope"]

        if right is None:
            steer = self.last_steer_cmd * 0.92 or self.args.search_steer
            speed = VEL_BUSCA
        else:
            steer = (right - target) * KP_OFFSET + slope * KP_SLOPE
            if left is not None and left > -50 and right > 44:
                steer += 0.34
            if lane["dash_offset"] is None and right > 78:
                steer += 0.16
            if lane["dash_offset"] is None and right > 98:
                steer += 0.26
            if right < 36:
                steer -= 0.22
            speed = VEL_CURVA if abs(steer) > 0.38 or lane["confidence"] < 0.55 else VEL_BASE

        steer = clamp(steer, -1.0, 1.0)
        speed = int(clamp(speed, 0, VEL_MAX))
        self.last_steer_cmd = steer
        return steer, speed

    def calculate_motor_speeds(self, speed, steer_cmd):
        turn = abs(steer_cmd)
        inner_factor = 1.0 - min(0.18, turn * 0.12)
        if steer_cmd < -0.05:
            return int(speed * inner_factor), speed
        if steer_cmd > 0.05:
            return speed, int(speed * inner_factor)
        return speed, speed

    def draw_debug(self, frame, lane, steer_cmd, servo_value, speeds, fps):
        roi_y = lane["roi_y"]
        height, width = frame.shape[:2]
        center_x = width // 2

        cv2.rectangle(frame, (0, roi_y), (width - 1, height - 1), (0, 180, 0), 1)
        cv2.line(frame, (center_x, roi_y), (center_x, height), (255, 160, 0), 1)

        for item in lane["right_points"]:
            x = int(center_x + item["offset"])
            y = int(roi_y + item["row"])
            cv2.circle(frame, (x, y), 4, (255, 255, 0), -1)

        for item in lane["left_points"]:
            x = int(center_x + item["offset"])
            y = int(roi_y + item["row"])
            cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

        target_x = int(center_x + lane["target"])
        cv2.line(frame, (target_x, roi_y), (target_x, height), (0, 255, 255), 1)

        lines = [
            f"right={fmt(lane['right_offset'])} left={fmt(lane['left_offset'])} dash={fmt(lane['dash_offset'])}",
            f"target={lane['target']:.1f} slope={lane['slope']:.2f} conf={lane['confidence']:.2f}",
            f"steer={steer_cmd:.2f} servo={servo_value:.2f}",
            f"motor L/R={speeds[0]}/{speeds[1]} fps={fps:.1f}",
        ]
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (8, 22 + i * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
            cv2.putText(frame, text, (8, 22 + i * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 1)
        return frame

    def log_status(self, lane, steer_cmd, servo_value, speeds, fps):
        now = time.time()
        if now - self.last_log < self.args.log_interval:
            return
        self.last_log = now
        print(
            f"frame={self.frame_count} "
            f"right={fmt(lane['right_offset'])} left={fmt(lane['left_offset'])} "
            f"target={lane['target']:.1f} steer={steer_cmd:.2f} servo={servo_value:.2f} "
            f"motor={speeds[0]}/{speeds[1]} conf={lane['confidence']:.2f} fps={fps:.1f}"
        )

    def run(self):
        print("\n--- INICIANDO NAVEGACAO AUTONOMA ---")
        print("Use Ctrl+C para parar. Com janela aberta, pressione q ou ESC.")
        self.set_motor_direction(forward=True)

        try:
            while self.running:
                start = time.time()
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    print("ERRO: falha ao capturar frame")
                    self.stop_motors()
                    time.sleep(0.05)
                    continue

                self.frame_count += 1
                lane = self.detect_lane(frame)
                steer_cmd, speed = self.controller(lane)
                servo_value = self.set_servo(steer_cmd)
                speeds = self.calculate_motor_speeds(speed, steer_cmd)
                self.set_motor_speeds(*speeds)

                fps = 1.0 / max(time.time() - start, 0.001)
                self.log_status(lane, steer_cmd, servo_value, speeds, fps)

                if self.display:
                    debug = self.draw_debug(frame.copy(), lane, steer_cmd, servo_value, speeds, fps)
                    cv2.imshow("camera", debug)
                    cv2.imshow("mask", lane["mask"])
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        print("Parada solicitada pela tecla")
                        break

                if self.args.loop_delay > 0:
                    time.sleep(self.args.loop_delay)

        except KeyboardInterrupt:
            print("\nInterrompido pelo usuario")
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n--- FINALIZANDO COM SEGURANCA ---")
        self.running = False
        try:
            self.stop_motors()
            time.sleep(0.15)
            if self.my_servo is not None:
                self.my_servo.value = SERVO_CENTRO
                time.sleep(0.25)
            if self.pwm_esq is not None:
                self.pwm_esq.stop()
            if self.pwm_dir is not None:
                self.pwm_dir.stop()
            if GPIO is not None and not self.dry_run:
                GPIO.cleanup()
            if self.cap is not None:
                self.cap.release()
            if self.display:
                cv2.destroyAllWindows()
        finally:
            print("Sistema finalizado")


def parse_args():
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
    parser.add_argument("--dry-run", action="store_true", help="Nao aciona GPIO/servo; usa somente camera e debug")
    parser.add_argument("--no-display", action="store_true", help="Roda sem janelas OpenCV, bom para SSH")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print("--- SISTEMA AUTONOMO ACKERMANN + CAMERA USB INICIADO ---")
    car = None
    try:
        car = AutonomousCar(args)
        time.sleep(1.0)
        car.run()
    except Exception as error:
        print(f"ERRO FATAL: {error}")
        if car is not None:
            car.cleanup()
        elif GPIO is not None and not args.dry_run:
            GPIO.cleanup()
        sys.exit(1)
