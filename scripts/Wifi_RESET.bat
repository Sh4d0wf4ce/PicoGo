@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo === [TRYB INTERNET - WI-FI] PRZYWRACANIE AUTOMATYCZNEGO IP (DHCP) ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$w = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -match '802.11|Wireless|Native' -or $_.MediaType -match '802.11' } | Select-Object -First 1; if ($w) { Write-Host ('Znaleziono karte Wi-Fi: ' + $w.Name) -ForegroundColor Cyan; netsh interface ipv4 set address name=\"$($w.Name)\" source=dhcp; netsh interface ipv4 set dnsservers name=\"$($w.Name)\" source=dhcp; Write-Host 'SUKCES: Przywrocono automatyczne IP (DHCP) oraz DNS na Wi-Fi!' -ForegroundColor Green } else { Write-Host 'BLAD: Nie wykryto karty Wi-Fi!' -ForegroundColor Red }"

echo.
pause