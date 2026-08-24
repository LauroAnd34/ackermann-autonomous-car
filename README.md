# Ackermann Autonomous Car Simulator

Simulador e codigo Raspberry Pi para um carrinho autonomo com direcao Ackermann, camera USB, servo de direcao e motores DC. O foco atual e a modalidade de placas na cidade interna da pista FIRA.

## Estrutura

- `ackermann-track-sim-web/`: simulador web em HTML/CSS/JS.
- `ackermann_robot_right_lane_controller.py`: codigo para Raspberry Pi com camera USB, GPIO, motores e servo.

## Rodar o simulador

```bash
cd ackermann-track-sim-web
python3 -m http.server 8121
```

Depois abra:

```text
http://127.0.0.1:8121/?scenario=city-right-turn
```

## O que o simulador faz

- Usa modelo bicicleta Ackermann.
- Mostra camera virtual e processamento binario da pista.
- Segue a faixa da direita pela linha continua direita.
- Reconhece placas com sentido de aproximacao.
- So executa conversao depois de passar da placa.
- Para se encontrar uma linha continua/bloqueio sem placa valida orientando a decisao.
- Permite criar geradores de placa no mapa e sortear placas permitidas.

## Codigo Raspberry Pi

Instale dependencias principais:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-rpi.gpio python3-gpiozero
```

Rode primeiro sem acionar motores/servo:

```bash
python3 ackermann_robot_right_lane_controller.py --dry-run
```

Rodar no carrinho:

```bash
python3 ackermann_robot_right_lane_controller.py --camera 0
```

Rodar sem janelas OpenCV, bom para SSH:

```bash
python3 ackermann_robot_right_lane_controller.py --camera 0 --no-display
```

## Pinagem padrao BCM

| Funcao | Pino BCM |
| --- | ---: |
| Servo direcao | 26 |
| Motor esquerdo IN1 | 17 |
| Motor esquerdo IN2 | 27 |
| Motor esquerdo PWM/ENA | 18 |
| Motor direito IN3 | 22 |
| Motor direito IN4 | 23 |
| Motor direito PWM/ENB | 19 |

Esses pinos podem ser alterados por argumento:

```bash
python3 ackermann_robot_right_lane_controller.py \
  --pin-servo 26 \
  --pin-left-in1 17 --pin-left-in2 27 --pin-left-pwm 18 \
  --pin-right-in1 22 --pin-right-in2 23 --pin-right-pwm 19
```

## Ajustes importantes

- `--white-threshold`: limiar da linha branca.
- `--search-steer`: direcao usada quando perde a linha.
- `--servo-min-pulse` e `--servo-max-pulse`: pulso fisico do servo.
- `VEL_BASE`, `VEL_CURVA`, `VEL_BUSCA`: velocidades PWM no arquivo Python.

## Observacao

O reconhecimento de placas real no Raspberry Pi ainda deve ser integrado com camera real/modelo ou classificacao por cor/formato. No simulador, as placas sao objetos virtuais com posicao e sentido para testar a logica de decisao.
