@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo === [TRYB INTERNET - WI-FI] PRZYWRACANIE AUTOMATYCZNEGO IP (DHCP) ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$w = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -match '802.11|Wireless|Native' -or $_.MediaType -match '802.11' } | Select-Object -First 1; if ($w) { Write-Host ('Znaleziono karte Wi-Fi: ' + $w.Name + ' (Status: ' + $w.Status + ')') -ForegroundColor Cyan; Remove-NetIPAddress -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -PolicyStore PersistentStore -Confirm:$false -ErrorAction SilentlyContinue; Remove-NetIPAddress -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue; Set-NetIPInterface -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -Dhcp Enabled -PolicyStore PersistentStore -ErrorAction SilentlyContinue; Set-NetIPInterface -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -Dhcp Enabled -ErrorAction SilentlyContinue; Set-DnsClientServerAddress -InterfaceIndex $w.ifIndex -ResetServerAddresses -ErrorAction SilentlyContinue; Write-Host 'SUKCES: Przywrocono automatyczne IP (DHCP) oraz DNS na karcie Wi-Fi!' -ForegroundColor Green } else { Write-Host 'BLAD: Nie wykryto karty Wi-Fi!' -ForegroundColor Red }"

echo.
pause