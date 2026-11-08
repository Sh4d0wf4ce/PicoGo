# app.py - Wersja 5.0: Autonomiczny robot (Wi-Fi + unikanie przeszkód)
# Zmiany względem 4.1:
#   - Usunięto jazdę po linii (TRSensor) na rzecz autonomicznego roamingu
#   - Robot jedzie prosto, wykrywa przeszkody ultradźwiękami
#   - Przy przeszkodzie: cofa się i skręca o LOSOWY kąt, potem jedzie dalej
#   - Po 3 kolejnych wykryciach robi dłuższy obrót (ciasny zakręt)
import network
import socket
import time
import json
import random
import machine
from machine import Pin
from Motor import PicoGo
from TRSensor import TRSensor  # zostawione dla komendy CALIBRATE

APP_VERSION = "5.0"

UDP_LISTEN_PORT = 5005
UDP_REPORT_PORT = 5006

# Inicjalizacja sprzętu
led = Pin("LED", Pin.OUT)
ROBOT_UID = machine.unique_id().hex()

M = PicoGo()
M.setMotor(0, 0)

Echo = Pin(15, Pin.IN, Pin.PULL_DOWN)
Trig = Pin(14, Pin.OUT)
Trig.value(0)

TRS = TRSensor()

# Ziarno losowości per-robot (żeby różne roboty skręcały różnie)
try:
    random.seed(int(ROBOT_UID, 16) & 0xFFFFFFFF)
except Exception:
    pass

# --- STAŁE PARAMETRY STEROWANIA ---
STOP_DISTANCE      = 25.0   # cm - poniżej tego dystansu unikamy przeszkody
MAX_FORWARD_SPEED  = 45     # górny limit prędkości jazdy
FORWARD_SPEED_MIN  = 25     # min. prędkość jazdy (pokonanie tarcia statycznego)
DIST_CHECK_MS      = 60     # co ile ms mierzyć odległość

TURN_SPEED    = 45          # moc skrętu w miejscu
TURN_MIN_MS   = 400         # losowy czas skrętu: od 0.4 s
TURN_MAX_MS   = 1500        #                       do 1.5 s
BACKOFF_MS    = 400         # czas cofania przed skrętem
BACKOFF_SPEED = 35
HARD_TURN_MS  = 1600        # dłuższy obrót po kilku kolejnych przeszkodach

# --- PARAMETRY ZMIENIANE NA ŻYWO PRZEZ WI-FI ---
params = {
    "mode": "IDLE",       # "IDLE" - stoi, "RUN" - autonomiczny roaming
    "freespeed": 35.0,    # prędkość jazdy do przodu
    "acc": 0.5,           # przyspieszenie (zachowane dla zgodności z UI)
    "brake": 0.8,         # hamowanie (jw.)
    "dist": 0.0,          # podgląd odległości
    "spd": 0.0            # podgląd prędkości
}


def dist():
    """Pomiar odległości HC-SR04 (cm). 100.0 przy braku echa/błędzie."""
    Trig.value(0)
    time.sleep_us(2)
    Trig.value(1)
    time.sleep_us(10)
    Trig.value(0)

    start = time.ticks_us()
    while Echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > 25000:
            return 100.0

    ts = time.ticks_us()
    while Echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), ts) > 25000:
            return 100.0
    te = time.ticks_us()

    pulse_us = time.ticks_diff(te, ts)
    if pulse_us < 100:            # <~1.7 cm -> traktujemy jako "brak przeszkody"
        return 100.0

    return (pulse_us * 0.034) / 2.0


def wykonaj_kalibracje():
    """Kalibracja czujników linii (na wypadek późniejszego użycia)."""
    print("[APP] Kalibracja czujnikow linii...")
    for i in range(100):
        if i < 25 or i >= 75:
            M.setMotor(30, -30)
        else:
            M.setMotor(-30, 30)
        TRS.calibrate()
    M.setMotor(0, 0)
    print("[APP] Kalibracja zakonczona.")


def zrob_kolka(czas_sekundy=2.0, predkosc=50):
    M.setMotor(predkosc, -predkosc)
    time.sleep(czas_sekundy)
    M.setMotor(0, 0)


def unikaj_przeszkody(hard=False):
    """Cofa się i skręca w miejscu o losowy kąt w losowym kierunku."""
    M.setMotor(0, 0)
    time.sleep_ms(80)     # chwila na zatrzymanie

    # Cofanie, żeby zrobić sobie miejsce na obrót
    M.setMotor(-BACKOFF_SPEED, -BACKOFF_SPEED)
    time.sleep_ms(BACKOFF_MS)
    M.setMotor(0, 0)
    time.sleep_ms(60)

    # Skręt w miejscu
    kierunek = random.choice([-1, 1])
    czas_ms = HARD_TURN_MS if hard else random.randint(TURN_MIN_MS, TURN_MAX_MS)

    print("[APP] Unikam: kierunek=%s, czas=%d ms"
          % ('L' if kierunek < 0 else 'P', czas_ms))

    M.setMotor(kierunek * TURN_SPEED, -kierunek * TURN_SPEED)
    time.sleep_ms(czas_ms)
    M.setMotor(0, 0)


def run():
    print(f"[APP v{APP_VERSION}] Start autonomicznego robota. ID: {ROBOT_UID}")
    wlan = network.WLAN(network.STA_IF)
    led.on()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_LISTEN_PORT))
    sock.setblocking(False)

    last_heartbeat = 0
    server_ip = '255.255.255.255'

    d = 100.0
    speed = 0.0
    last_dist_check = 0
    consecutive_obstacles = 0

    while True:
        loop_start = time.ticks_ms()

        # ============ 1. KOMUNIKACJA WI-FI (nieblokująca) ============
        if wlan.isconnected():
            # A) HELLO co 2 s
            if time.ticks_diff(loop_start, last_heartbeat) > 2000:
                last_heartbeat = loop_start
                params["dist"] = round(d, 1)
                params["spd"]  = round(speed, 1)
                meldunek = {
                    "type": "HELLO",
                    "id":   ROBOT_UID,
                    "ver":  APP_VERSION,
                    "params": params
                }
                try:
                    sock.sendto(json.dumps(meldunek).encode('utf-8'),
                                (server_ip, UDP_REPORT_PORT))
                except Exception:
                    pass

            # B) Odbieranie komend
            while True:
                try:
                    data, addr = sock.recvfrom(512)
                    server_ip = addr[0]
                    msg = json.loads(data.decode('utf-8'))
                    cmd = msg.get("cmd")

                    if cmd == "OTA_UPDATE":
                        M.setMotor(0, 0)
                        sock.close()
                        return ("OTA", server_ip, msg.get("port", 8000))

                    elif cmd == "SPIN":
                        czas = float(msg.get("duration", 2.0))
                        moc  = int(msg.get("power", 50))
                        zrob_kolka(czas, moc)
                        last_heartbeat = 0

                    elif cmd == "CALIBRATE":
                        wykonaj_kalibracje()
                        last_heartbeat = 0

                    if "params" in msg:
                        params.update(msg["params"])
                        if params["mode"] != "RUN":
                            M.setMotor(0, 0)
                            speed = 0.0
                            consecutive_obstacles = 0
                        last_heartbeat = 0

                except OSError:
                    break

        # ============ 2. GŁÓWNA LOGIKA JAZDY ============
        if params["mode"] != "RUN":
            M.setMotor(0, 0)
            speed = 0.0
            consecutive_obstacles = 0
        else:
            now = time.ticks_ms()

            # Pomiar odległości co ~60 ms
            if time.ticks_diff(now, last_dist_check) >= DIST_CHECK_MS:
                last_dist_check = now
                d = dist()

            if d <= STOP_DISTANCE:
                # --- Wykryto przeszkodę ---
                consecutive_obstacles += 1
                speed = 0.0
                M.setMotor(0, 0)

                hard = consecutive_obstacles >= 3
                unikaj_przeszkody(hard=hard)
                if hard:
                    consecutive_obstacles = 0

                # Świeży pomiar po manewrze
                d = dist()
                last_dist_check = time.ticks_ms()
            else:
                # --- Droga wolna: jedź do przodu ---
                consecutive_obstacles = 0

                free_spd = float(params["freespeed"])
                target   = max(FORWARD_SPEED_MIN,
                               min(free_spd, MAX_FORWARD_SPEED))

                # Łagodna rampa prędkości
                if speed < FORWARD_SPEED_MIN:
                    speed = float(FORWARD_SPEED_MIN)
                elif speed < target:
                    speed = min(speed + float(params["acc"]), target)
                elif speed > target:
                    speed = max(speed - float(params["brake"]), target)

                spd_int = int(max(0, min(100, speed)))
                M.setMotor(spd_int, spd_int)

        # ============ 3. Taktowanie pętli 20 ms (50 Hz) ============
        elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
        if elapsed < 20:
            time.sleep_ms(20 - elapsed)