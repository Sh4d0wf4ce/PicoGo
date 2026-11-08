@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo === [TRYB ROBOTY - ETHERNET] KONFIGURACJA KARTY PRZEWODOWEJ ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$a = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -eq '802.3' -or $_.MediaType -eq '802.3' } | Select-Object -First 1; if ($a) { Write-Host ('Znaleziono karte: ' + $a.Name) -ForegroundColor Cyan; netsh interface ipv4 set address name=\"$($a.Name)\" static 192.168.1.1 255.255.255.0 none; Write-Host 'SUKCES: Ustawiono statyczny adres IP 192.168.1.1' -ForegroundColor Green } else { Write-Host 'BLAD: Nie wykryto karty Ethernet!' -ForegroundColor Red }"

netsh advfirewall firewall delete rule name="PicoGo_Panel" >nul 2>&1
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=UDP localport=67,68,5005,5006 >nul
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=TCP localport=8000 >nul

echo Odblokowano porty komunikacyjne w Zaporze Windows.
echo.
pause