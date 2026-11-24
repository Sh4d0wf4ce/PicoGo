# apps/app_vision.py - Sterowanie wizyjne z kamery ArUco + natywna obsługa OTA przez main.py
import socket
import json
import time
import machine
from Motor import PicoGo

APP_VERSION = "vision_1.1"

UDP_LISTEN_PORT = 5005
UDP_TELEMETRY_PORT = 5006
DEFAULT_SERVER_IP = "192.168.1.1"

# Watchdog: jeśli przez 400 ms nie przyjdzie komenda z kamery, zatrzymaj silniki
WATCHDOG_TIMEOUT_MS = 400


def apply_differential_drive(motor, speed, turn):
    """
    Przelicza znormalizowane wartości (speed, turn) z regulatora kamery
    na sygnał PWM (-100..100) dla lewego i prawego koła.
    """
    if abs(speed) < 0.01 and abs(turn) < 0.01:
        motor.setMotor(0, 0)
        return 0.0

    left_val = (speed + turn) * 100.0
    right_val = (speed - turn) * 100.0

    left_pwm = int(max(-100, min(100, round(left_val))))
    right_pwm = int(max(-100, min(100, round(right_val))))

    motor.setMotor(left_pwm, right_pwm)
    return (abs(left_pwm) + abs(right_pwm)) / 2.0


def run(wlan=None):
    M = PicoGo()
    M.setMotor(0, 0)
    uid = machine.unique_id().hex()

    # Gniazdo odbierające komendy (port 5005)
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(("0.0.0.0", UDP_LISTEN_PORT))
    rx_sock.setblocking(False)

    # Gniazdo wysyłające telemetrię HELLO (port 5006)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    server_ip = DEFAULT_SERVER_IP
    mode = "VISION_IDLE"
    current_spd = 0.0
    last_cmd_ms = time.ticks_ms()
    last_hello_ms = 0
    last_seq = None

    print(f"[APP_VISION] Uruchomiono v{APP_VERSION} (UID: {uid})")

    while True:
        now = time.ticks_ms()

        # 1. Odbiór wszystkich oczekujących pakietów UDP
        while True:
            try:
                data, addr = rx_sock.recvfrom(512)
            except OSError:
                break

            server_ip = addr[0]

            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            # Pomijanie duplikatów komend z numerem seq (broadcast + unicast)
            seq = msg.get("seq")
            if seq is not None:
                if seq == last_seq:
                    continue
                last_seq = seq

            cmd = msg.get("cmd", "")
            msg_type = msg.get("type", "")

            # Przekazanie OTA do bootloadera main.py (identycznie jak w app_idm.py - bez machine.reset!)
            if cmd == "OTA_UPDATE":
                M.setMotor(0, 0)
                rx_sock.close()
                tx_sock.close()
                return ("OTA", server_ip, msg.get("port", 8000))

            # Obsługa identyfikacji (krótki bączek)
            elif cmd == "SPIN" or msg_type == "identify":
                dur = float(msg.get("duration", 0.4))
                pwr = int(msg.get("power", 40))
                M.setMotor(pwr, -pwr)
                time.sleep(dur)
                M.setMotor(0, 0)
                last_cmd_ms = time.ticks_ms()

            # Komenda jazdy z kamery ArUco
            elif cmd == "VISION_DRIVE" or msg_type == "cmd":
                spd = float(msg.get("speed", 0.0))
                trn = float(msg.get("turn", 0.0))
                current_spd = apply_differential_drive(M, spd, trn)
                last_cmd_ms = now
                mode = "VISION_RUN" if current_spd > 0 else "VISION_IDLE"

            # Obsługa przycisku STOP (Spacja)
            elif "params" in msg:
                p = msg["params"]
                if p.get("mode") == "IDLE":
                    M.setMotor(0, 0)
                    current_spd = 0.0
                    mode = "VISION_IDLE"

        # 2. Watchdog bezpieczeństwa
        if mode == "VISION_RUN" and time.ticks_diff(now, last_cmd_ms) > WATCHDOG_TIMEOUT_MS:
            M.setMotor(0, 0)
            current_spd = 0.0
            mode = "VISION_IDLE"

        # 3. Wysyłanie meldunku HELLO co 800 ms
        if time.ticks_diff(now, last_hello_ms) >= 800:
            last_hello_ms = now
            hello_pkt = json.dumps({
                "type": "HELLO",
                "id": uid,
                "ver": APP_VERSION,
                "params": {
                    "mode": mode,
                    "freespeed": 0.0,
                    "acc": 0.0,
                    "brake": 0.0,
                    "dist": 0.0,
                    "spd": round(current_spd, 1)
                }
            }).encode("utf-8")
            try:
                tx_sock.sendto(hello_pkt, (server_ip, UDP_TELEMETRY_PORT))
                tx_sock.sendto(hello_pkt, ("255.255.255.255", UDP_TELEMETRY_PORT))
            except Exception:
                pass

        time.sleep_ms(5)