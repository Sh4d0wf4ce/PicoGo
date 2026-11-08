@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo === [TRYB INTERNET - ETHERNET] PRZYWRACANIE AUTOMATYCZNEGO IP (DHCP) ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$a = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -eq '802.3' -or $_.MediaType -eq '802.3' } | Select-Object -First 1; if ($a) { Write-Host ('Znaleziono karte: ' + $a.Name) -ForegroundColor Cyan; netsh interface ipv4 set address name=\"$($a.Name)\" source=dhcp; netsh interface ipv4 set dnsservers name=\"$($a.Name)\" source=dhcp; Write-Host 'SUKCES: Przywrocono automatyczne IP (DHCP) oraz DNS!' -ForegroundColor Green } else { Write-Host 'BLAD: Nie wykryto karty Ethernet!' -ForegroundColor Red }"

echo.
pause