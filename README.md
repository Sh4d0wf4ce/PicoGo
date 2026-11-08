# PicoGo Control Panel

System sterowania, telemetrii i bezprzewodowej aktualizacji kodu (**OTA**) dla floty robotów **Raspberry Pi Pico 2 W (PicoGo)** z wykorzystaniem dedykowanego punktu dostępowego **Ubiquiti UniFi** oraz aplikacji desktopowej w **C++20 / Qt6**.

---

## 📂 Struktura projektu

* **`firmware/`** – Kod bazowy wgrywany jednorazowo przez USB (Thonny) na każdego robota:
  * `main.py` – Bootloader Wi-Fi (`PicoNet`) z obsługą bezprzewodowych aktualizacji OTA i trybem ratunkowym (`RECOVERY`).
  * `lib/` – Wymagane sterowniki sprzętowe (`Motor.py`, `TRSensor.py` v2, `Ultrasonic_Ranging.py`).
* **`apps/`** – Wymienne programy sterujące wgrywane bezprzewodowo (OTA) z poziomu aplikacji desktopowej:
  * `app_idm.py` – Jazda po torze z zachowaniem odstępu według modelu **IDM (Intelligent Driver Model)**.
  * `app_autonomous_avoid.py` – Autonomiczna jazda swobodna z omijaniem przeszkód.
* **`control_panel/`** – Kod źródłowy aplikacji **PicoGo Control Panel** (C++20, Qt6 Widgets + Network, wbudowany serwer DHCP, serwer HTTP OTA i sterowanie UDP).
* **`scripts/`** – Skrypty `.bat` (z automatycznymi uprawnieniami Administratora) konfigurujące kartę sieciową Ethernet/Wi-Fi na komputerze operatora.

---

##  Szybki start (Sterowanie robotami)

### 1. Podłączenie Access Pointa Ubiquiti
1. Podłącz dysk **Ubiquiti** kablem Ethernet do gniazda **PoE (z symbolem pioruna)** w zasilaczu PoE.
2. Gniazdo **LAN (ze strzałkami)** w zasilaczu PoE połącz drugim kablem Ethernet z gniazdem Ethernet w laptopie (lub połącz laptopa bezprzewodowo z siecią Wi-Fi `PicoNet`, hasło: `BozonHiggsa`).

### 2. Konfiguracja sieci w systemie Windows
* **Jeśli laptop jest podpięty kablem Ethernet do zasilacza Ubiquiti:**
  Uruchom dwuklikiem `scripts/Ethernet_START.bat` *(po zakończeniu pracy przywróć ustawienia domowe plikiem `Ethernet_RESET.bat`)*.
* **Jeśli laptop łączy się z dyskiem Ubiquiti przez Wi-Fi (`PicoNet`):**
  Połącz się z Wi-Fi `PicoNet`, a następnie uruchom `scripts/WiFi_START.bat` *(po zakończeniu testów uruchom `WiFi_RESET.bat`)*.

### 3. Uruchomienie panelu sterowania
1. Uruchom `PicoGoControlPanel.exe`.
2. Włącz zasilanie na robotach – wbudowany w aplikację serwer DHCP automatycznie przydzieli im adresy IP (`192.168.1.21+`), a roboty pojawią się w tabeli ze statusem `🟢 ONLINE`.

---

## 🛠️ Kompilacja aplikacji C++ / Qt6 ze źródeł

Wymagania: **CMake**, kompilator C++20 (**MinGW-w64 / GCC** lub MSVC) oraz **Qt 6** (`Qt6Widgets`, `Qt6Network`).

```bash
cd control_panel
cmake -S . -B build -G Ninja
cmake --build build