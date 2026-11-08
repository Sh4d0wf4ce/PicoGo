import network
import socket
import time
import json
import random
import os
import sys
import machine

WIFI_SSID = "PicoNet"
WIFI_PASS = "BozonHiggsa"

led = machine.Pin("LED", machine.Pin.OUT)

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if wlan.isconnected() and wlan.ifconfig()[0] != '0.0.0.0':
        led.on()
        print("[MAIN] Wi-Fi juz aktywne! IP:", wlan.ifconfig()[0])
        return True
    
    print("[MAIN] Laczenie z Wi-Fi:", WIFI_SSID)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    
    for i in range(30):
        if wlan.isconnected() and wlan.ifconfig()[0] != '0.0.0.0':
            led.on()
            print("[MAIN] Polaczono! Moje IP:", wlan.ifconfig()[0])
            return True
        led.toggle()
        time.sleep(0.5)
        
    led.off()
    print("[MAIN] Brak Wi-Fi - uruchamiam program w trybie offline.")
    return False


def download_new_app(server_ip, server_port=8000):
    time.sleep(random.uniform(0.1, 0.8))
    print(f"[OTA] Pobieranie nowego app.py z {server_ip}:{server_port}...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((server_ip, server_port))
        request = f"GET /app.py HTTP/1.0\r\nHost: {server_ip}\r\nConnection: close\r\n\r\n"
        s.send(request.encode('utf-8'))
        
        response = b""
        while True:
            chunk = s.recv(1024)
            if not chunk:
                break
            response += chunk
        s.close()
        
        parts = response.split(b"\r\n\r\n", 1)
        if len(parts) == 2 and b"200 OK" in parts[0]:
            code_bytes = parts[1]
            if len(code_bytes) > 50:
                with open("app.py", "wb") as f:
                    f.write(code_bytes)
                    f.flush()
                os.sync()
                print(f"[OTA] SUKCES! Zapisano nowy app.py ({len(code_bytes)} B). Przeladowuje w locie!")
                for _ in range(6):
                    led.toggle()
                    time.sleep(0.1)
                led.on()
                return True
        print("[OTA] Blad odpowiedzi serwera HTTP.")
    except Exception as e:
        print("[OTA] Blad pobierania pliku:", e)
    return False

def wait_for_recovery_ota():
    print("[RECOVERY] Czekam na poprawiony app.py (OTA_UPDATE) na porcie 5005...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 5005))
    sock.settimeout(1.0)
    while True:
        led.toggle()
        try:
            data, addr = sock.recvfrom(512)
            msg = json.loads(data.decode('utf-8'))
            if msg.get("cmd") == "OTA_UPDATE":
                sock.close()
                if download_new_app(addr[0], msg.get("port", 8000)):
                    return
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind(('0.0.0.0', 5005))
                sock.settimeout(1.0)
        except OSError:
            try:
                uid = machine.unique_id().hex()
                sos = json.dumps({"type": "HELLO", "id": uid, "ver": "BLAD_KODU"}).encode('utf-8')
                sock.sendto(sos, ('255.255.255.255', 5006))
            except Exception:
                pass

# --- GLOWNY START I PETLA PRZELADOWYWANIA ---
connect_wifi()

while True:
    try:
        if 'app' in sys.modules:
            del sys.modules['app']
        import gc
        gc.collect()
        
        print("[MAIN] Uruchamiam app.py...")
        import app
        wynik = app.run()
        
        if isinstance(wynik, tuple) and wynik[0] == "OTA":
            download_new_app(wynik[1], wynik[2])
            
    except Exception as e:
        print("[MAIN] BLAD KRYTYCZNY w pliku app.py:", e)
        wait_for_recovery_ota()