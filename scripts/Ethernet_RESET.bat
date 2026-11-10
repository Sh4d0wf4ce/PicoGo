@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

rem Adres komputera w sieci robotow (maska /24).
rem Musi zgadzac sie z SUBNET_PREFIX w control_panel/MainWindow.h
set "ROBOT_IP=192.168.1.1"

echo === [TRYB INTERNET - ETHERNET] PRZYWRACANIE AUTOMATYCZNEGO IP (DHCP) ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$ip = '%ROBOT_IP%'; $names = @(@(Get-NetIPAddress -IPAddress $ip -ErrorAction SilentlyContinue) + @(Get-NetIPAddress -IPAddress $ip -PolicyStore PersistentStore -ErrorAction SilentlyContinue) | Select-Object -ExpandProperty InterfaceAlias -Unique | Where-Object { (Get-NetAdapter -Name $_ -ErrorAction SilentlyContinue).PhysicalMediaType -eq '802.3' }); if ($names.Count -eq 0) { $a = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -eq '802.3' -or $_.MediaType -eq '802.3' } | Select-Object -First 1; if ($a) { $names = @($a.Name) } }; if ($names.Count -gt 0) { foreach ($n in $names) { Write-Host ('Znaleziono karte: ' + $n) -ForegroundColor Cyan; netsh interface ipv4 set address name=\"$n\" source=dhcp; netsh interface ipv4 set dnsservers name=\"$n\" source=dhcp }; Write-Host 'SUKCES: Przywrocono automatyczne IP (DHCP) oraz DNS!' -ForegroundColor Green } else { Write-Host 'BLAD: Nie wykryto karty Ethernet!' -ForegroundColor Red }"

echo.
pause