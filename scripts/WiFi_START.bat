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

echo === [TRYB ROBOTY - WI-FI] KONFIGURACJA KARTY BEZPRZEWODOWEJ ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$ip = '%ROBOT_IP%'; $w = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -match '802.11|Wireless|Native' -or $_.MediaType -match '802.11' } | Sort-Object { $_.Status -ne 'Up' } | Select-Object -First 1; if ($w) { Write-Host ('Znaleziono karte Wi-Fi: ' + $w.Name + ' (Status: ' + $w.Status + ')') -ForegroundColor Cyan; $sel = $w.ifIndex; $other = @(Get-NetIPAddress -IPAddress $ip -ErrorAction SilentlyContinue) + @(Get-NetIPAddress -IPAddress $ip -PolicyStore PersistentStore -ErrorAction SilentlyContinue) | Where-Object { $_.InterfaceIndex -ne $sel } | Select-Object -ExpandProperty InterfaceIndex -Unique; foreach ($i in $other) { Write-Host ('Zdejmuje adres ' + $ip + ' z innej karty: ' + (Get-NetAdapter -InterfaceIndex $i -ErrorAction SilentlyContinue).Name) -ForegroundColor Yellow; Remove-NetIPAddress -InterfaceIndex $i -IPAddress $ip -Confirm:$false -ErrorAction SilentlyContinue; Remove-NetIPAddress -InterfaceIndex $i -IPAddress $ip -PolicyStore PersistentStore -Confirm:$false -ErrorAction SilentlyContinue; Set-NetIPInterface -InterfaceIndex $i -AddressFamily IPv4 -Dhcp Enabled -ErrorAction SilentlyContinue }; Set-NetIPInterface -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -Dhcp Disabled -PolicyStore PersistentStore -ErrorAction SilentlyContinue; Set-NetIPInterface -InterfaceIndex $w.ifIndex -AddressFamily IPv4 -Dhcp Disabled -ErrorAction SilentlyContinue; netsh interface ipv4 set address name=\"$($w.Name)\" source=static address=$ip mask=255.255.255.0 store=persistent; Write-Host ('SUKCES: Zapisano na stale adres IP ' + $ip + ' na karcie Wi-Fi!') -ForegroundColor Green; if ($w.Status -ne 'Up') { Write-Host 'UWAGA: karta Wi-Fi nie jest polaczona - polacz sie z siecia PicoNet.' -ForegroundColor Yellow } } else { Write-Host 'BLAD: Nie wykryto karty Wi-Fi!' -ForegroundColor Red }"

netsh advfirewall firewall delete rule name="PicoGo_Panel" >nul 2>&1
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=UDP localport=67,68,5005,5006 >nul
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=TCP localport=8000 >nul

echo Odblokowano porty komunikacyjne w Zaporze Windows.

rem Jesli przy pierwszym uruchomieniu panelu zamknieto okno Zapory Windows, system utworzyl
rem regule BLOKUJACA program. Taka regula wygrywa z regulami portow powyzej, wiec ja usuwamy.
powershell -NoProfile -ExecutionPolicy Bypass -Command "$blk = @(Get-NetFirewallRule -Action Block -Direction Inbound -ErrorAction SilentlyContinue | Where-Object { ($_ | Get-NetFirewallApplicationFilter).Program -like '*PicoGoControlPanel.exe' }); if ($blk.Count -gt 0) { $blk | Remove-NetFirewallRule; Write-Host ('Usunieto reguly Zapory blokujace PicoGoControlPanel.exe: ' + $blk.Count) -ForegroundColor Yellow }"

echo.
pause