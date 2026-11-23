# apps/app_vision.py - Sterowanie wizyjne z kamery ArUco + pełna obsługa OTA i telemetrii
import socket
import json
import time
import machine
import ubinascii
from Motor import PicoGo

APP_VERSION = "vision_1.0"

UDP_LISTEN_PORT = 5005
UDP_TELEMETRY_PORT = 5006
SERVER_IP = "192.168.1.1"

# Watchdog: jeśli przez 400 ms nie przyjdzie komenda z kamery, zatrzymaj silniki
WATCHDOG_TIMEOUT_MS = 400


def get_uid():
    return ubinascii.hexlify(machine.unique_id()).decode()


def perform_ota_update(server_ip, port):
    print(f"[OTA] Pobieranie nowego kodu z {server_ip}:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    try:
        s.connect((server_ip, int(port)))
        s.send(b"GET /app.py HTTP/1.0\r\nHost: " + server_ip.encode() + b"\r\n\r\n")
        chunks = []
        while True:
            data = s.recv(1024)
            if not data:
                break
            chunks.append(data)
        s.close()
        raw = b"".join(chunks)
        header_end = raw.find(b"\r\n\r\n")
        if header_end != -1:
            body = raw[header_end + 4:]
            if len(body) > 50:
                with open("app.py.tmp", "wb") as f:
                    f.write(body)
                import os
                os.rename("app.py.tmp", "app.py")
                print("[OTA] Sukces! Restartuję robota...")
                time.sleep(0.3)
                machine.reset()
    except Exception as e:
        print("[OTA BŁĄD]", e)
    finally:
        try:
            s.close()
        except Exception:
            pass


def apply_differential_drive(motor, speed, turn):
    """
    Przelicza znormalizowane wartości (speed, turn) z regulatora kamery
    na sygnał PWM (-100..100) dla lewego i prawego koła.
    """
    if abs(speed) < 0.01 and abs(turn) < 0.01:
        motor.stop()
        return 0.0

    # W układzie współrzędnych obrazu (oś Y skierowana w dół) dodatni skręt (turn > 0)
    # oznacza obrót zgodnie z ruchem wskazówek zegara (lewe koło szybciej, prawe wolniej)
    left_val = (speed + turn) * 100.0
    right_val = (speed - turn) * 100.0

    # Nasycenie do zakresu [-100, 100]
    left_pwm = int(max(-100, min(100, round(left_val))))
    right_pwm = int(max(-100, min(100, round(right_val))))

    motor.setMotor(left_pwm, right_pwm)
    return (abs(left_pwm) + abs(right_pwm)) / 2.0


def run(wlan=None):
    M = PicoGo()
    M.stop()
    uid = get_uid()

    # Gniazdo odbierające komendy (port 5005)
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(("0.0.0.0", UDP_LISTEN_PORT))
    rx_sock.setblocking(False)

    # Gniazdo wysyłające telemetrię HELLO (port 5006)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    mode = "VISION_IDLE"
    current_spd = 0.0
    last_cmd_ms = time.ticks_ms()
    last_hello_ms = 0

    print(f"[APP_VISION] Uruchomiono v{APP_VERSION} (UID: {uid})")

    while True:
        now = time.ticks_ms()

        # 1. Odbiór wszystkich oczekujących pakietów UDP (bierzemy najświeższy)
        while True:
            try:
                data, addr = rx_sock.recvfrom(512)
            except OSError:
                break

            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            cmd = msg.get("cmd", "")
            msg_type = msg.get("type", "")

            # Obsługa bezprzewodowej aktualizacji OTA z naszego panelu C++
            if cmd == "OTA_UPDATE":
                M.stop()
                port = msg.get("port", 8000)
                perform_ota_update(addr[0], port)

            # Obsługa identyfikacji (krótki bączek z Zakładki 1 lub Zakładki 2)
            elif cmd == "SPIN" or msg_type == "identify":
                dur = float(msg.get("duration", 0.4))
                pwr = int(msg.get("power", 40))
                M.setMotor(pwr, -pwr)
                time.sleep(dur)
                M.stop()
                last_cmd_ms = time.ticks_ms()

            # Komenda jazdy z kamery ArUco (nasz format VISION_DRIVE oraz format profesora type=="cmd")
            elif cmd == "VISION_DRIVE" or msg_type == "cmd":
                spd = float(msg.get("speed", 0.0))
                trn = float(msg.get("turn", 0.0))
                current_spd = apply_differential_drive(M, spd, trn)
                last_cmd_ms = now
                mode = "VISION_RUN" if current_spd > 0 else "VISION_IDLE"

            # Obsługa przycisku STOP (Spacja) z naszego panelu C++
            elif "params" in msg:
                p = msg["params"]
                if p.get("mode") == "IDLE":
                    M.stop()
                    current_spd = 0.0
                    mode = "VISION_IDLE"

        # 2. Watchdog bezpieczeństwa (zatrzymaj robota, gdy kamera przestanie nadawać)
        if mode == "VISION_RUN" and time.ticks_diff(now, last_cmd_ms) > WATCHDOG_TIMEOUT_MS:
            M.stop()
            current_spd = 0.0
            mode = "VISION_IDLE"

        # 3. Wysyłanie meldunku HELLO co 800 ms do aplikacji C++
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
                tx_sock.sendto(hello_pkt, (SERVER_IP, UDP_TELEMETRY_PORT))
                tx_sock.sendto(hello_pkt, ("255.255.255.255", UDP_TELEMETRY_PORT))
            except Exception:
                pass

        time.sleep_ms(5)