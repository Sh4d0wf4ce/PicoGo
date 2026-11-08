# app.py - Wersja 4.1: IDM + Wi-Fi (bez czujnika gestów IR, z zabezpieczeniem US)
import network
import socket
import time
import json
import machine
from machine import Pin
from TRSensor import TRSensor
from Motor import PicoGo

APP_VERSION = "4.1"

UDP_LISTEN_PORT = 5005
UDP_REPORT_PORT = 5006

# Inicjalizacja sprzętu
led = Pin("LED", Pin.OUT)
ROBOT_UID = machine.unique_id().hex()

M = PicoGo()
M.setMotor(0, 0)

# Dodano PULL_DOWN na Echo, aby zapobiec fałszywym odczytom 0 cm!
Echo = Pin(15, Pin.IN, Pin.PULL_DOWN)
Trig = Pin(14, Pin.OUT)
Trig.value(0)

TRS = TRSensor()

# --- STAŁE PARAMETRY STEROWANIA ---
STOP_DISTANCE = 12.0
FREE_DISTANCE = 35.0
START_SPEED = 22.0

KP = 0.014
KD = 0.0003

MIN_SPEED = 0
MAX_SPEED = 60

# --- PARAMETRY ZMIENIANE NA ŻYWO PRZEZ WI-FI ---
params = {
    "mode": "IDLE",      # "IDLE" (stoi) lub "RUN" (jedzie wg modelu IDM)
    "freespeed": 35.0,
    "acc": 0.5,
    "brake": 0.8,
    "dist": 0.0,         # Podgląd zmierzonej odległości w tabeli serwera
    "spd": 0.0           # Podgląd aktualnej prędkości w tabeli serwera
}

def dist():
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
    # Jeśli impuls trwał krócej niż 100 us (< 1.7 cm), to błąd odczytu -> droga wolna
    if pulse_us < 100:
        return 100.0
        
    return (pulse_us * 0.034) / 2.0

def target_speed_from_distance(d):
    free_spd = float(params["freespeed"])
    if d <= STOP_DISTANCE:
        return 0.0
    if d >= FREE_DISTANCE:
        return free_spd
    return free_spd * (d - STOP_DISTANCE) / (FREE_DISTANCE - STOP_DISTANCE)

def wykonaj_kalibracje():
    print("[APP] Kalibracja czujnikow linii...")
    for i in range(100):
        if i < 25 or i >= 75:
            M.setMotor(30, -30)
        else:
            M.setMotor(-30, 30)
        TRS.calibrate()
    M.setMotor(0, 0)
    print("[APP] Kalibracja zakonczona:", TRS.calibratedMin, TRS.calibratedMax)

def zrob_kolka(czas_sekundy=2.0, predkosc=50):
    M.setMotor(predkosc, -predkosc)
    time.sleep(czas_sekundy)
    M.setMotor(0, 0)

def run():
    print(f"[APP v{APP_VERSION}] Start programu IDM. ID: {ROBOT_UID}")
    wlan = network.WLAN(network.STA_IF)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_LISTEN_PORT))
    sock.setblocking(False)
    
    last_heartbeat = 0
    server_ip = '255.255.255.255'
    
    # Zmienne stanu modelu IDM oraz regulatora PD
    speed = 0.0
    last_err = 0
    last_time = time.ticks_us()
    last_dir = 0
    d = 100.0
    
    while True:
        loop_start = time.ticks_ms()
        now_us = time.ticks_us()
        
        # 1. KOMUNIKACJA WI-FI (Nieblokująca)
        if wlan.isconnected():
            # A) Meldunek HELLO do serwera co 2 sekundy
            if time.ticks_diff(loop_start, last_heartbeat) > 2000:
                last_heartbeat = loop_start
                params["dist"] = round(d, 1)
                params["spd"] = round(speed, 1)
                meldunek = {
                    "type": "HELLO",
                    "id": ROBOT_UID,
                    "ver": APP_VERSION,
                    "params": params
                }
                try:
                    sock.sendto(json.dumps(meldunek).encode('utf-8'), (server_ip, UDP_REPORT_PORT))
                except Exception:
                    pass

            # B) Odbieranie komend z serwera
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
                        moc = int(msg.get("power", 50))
                        zrob_kolka(czas, moc)
                        last_time = time.ticks_us()
                        last_heartbeat = 0
                        
                    elif cmd == "CALIBRATE":
                        wykonaj_kalibracje()
                        last_time = time.ticks_us()
                        last_heartbeat = 0
                        
                    if "params" in msg:
                        params.update(msg["params"])
                        if params["mode"] != "RUN":
                            speed = 0.0
                            M.setMotor(0, 0)
                        last_time = time.ticks_us()
                        last_heartbeat = 0
                        
                except OSError:
                    break

        # 2. GŁÓWNA LOGIKA JAZDY (Gdy mode == "RUN")
        if params["mode"] != "RUN":
            M.setMotor(0, 0)
            speed = 0.0
            last_time = now_us
        else:
            # Odczyt linii
            position, sensors = TRS.readLine()
            
            # Awaryjne szukanie linii (gdy wszystkie czujniki widzą czerń lub wypadł)
            if sum(sensors) > 4000:
                if last_dir < 0:
                    M.setMotor(-30, 30)
                else:
                    M.setMotor(30, -30)
                last_time = now_us
            else:
                # Obliczenie uchybu linii
                err = position - 2000
                if err < -150:
                    last_dir = -1
                elif err > 150:
                    last_dir = 1
                    
                dt = time.ticks_diff(now_us, last_time) / 1000000.0
                if dt <= 0 or dt > 0.5:
                    dt = 0.02
                
                der = (err - last_err) / dt
                last_err = err
                last_time = now_us
                
                corr = (KP * err) + (KD * der)

                # Pomiar odległości i docelowa prędkość z modelu
                d = dist()
                target_speed = target_speed_from_distance(d)
                
                acc_val = float(params["acc"])
                brake_val = float(params["brake"])
                
                # Rampa prędkości z pokonaniem tarcia statycznego
                if target_speed > 0:
                    if speed < START_SPEED:
                        speed = START_SPEED
                    elif speed < target_speed:
                        speed = min(speed + acc_val, target_speed)
                    elif speed > target_speed:
                        speed = max(speed - brake_val, target_speed)
                else:
                    speed = max(speed - brake_val, 0.0)
                    
                # Odcięcie zasilania przy zatrzymaniu lub wysterowanie kół
                if speed <= 0.5:
                    M.setMotor(0, 0)
                else:
                    left_speed = speed + corr
                    right_speed = speed - corr
                    
                    left_speed = int(max(MIN_SPEED, min(MAX_SPEED, left_speed)))
                    right_speed = int(max(MIN_SPEED, min(MAX_SPEED, right_speed)))
                    
                    M.setMotor(left_speed, right_speed)

        # 3. Sztywne taktowanie pętli 20 ms (50 Hz)
        elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
        if elapsed < 20:
            time.sleep_ms(20 - elapsed)