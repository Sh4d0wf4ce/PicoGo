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

echo === [TRYB ROBOTY - ETHERNET] KONFIGURACJA KARTY PRZEWODOWEJ ===

powershell -NoProfile -ExecutionPolicy Bypass -Command "$ip = '%ROBOT_IP%'; $a = Get-NetAdapter -Physical | Where-Object { $_.PhysicalMediaType -eq '802.3' -or $_.MediaType -eq '802.3' } | Sort-Object { $_.Status -ne 'Up' } | Select-Object -First 1; if ($a) { Write-Host ('Znaleziono karte: ' + $a.Name + ' (Status: ' + $a.Status + ')') -ForegroundColor Cyan; $sel = $a.ifIndex; $other = @(Get-NetIPAddress -IPAddress $ip -ErrorAction SilentlyContinue) + @(Get-NetIPAddress -IPAddress $ip -PolicyStore PersistentStore -ErrorAction SilentlyContinue) | Where-Object { $_.InterfaceIndex -ne $sel } | Select-Object -ExpandProperty InterfaceIndex -Unique; foreach ($i in $other) { Write-Host ('Zdejmuje adres ' + $ip + ' z innej karty: ' + (Get-NetAdapter -InterfaceIndex $i -ErrorAction SilentlyContinue).Name) -ForegroundColor Yellow; Remove-NetIPAddress -InterfaceIndex $i -IPAddress $ip -Confirm:$false -ErrorAction SilentlyContinue; Remove-NetIPAddress -InterfaceIndex $i -IPAddress $ip -PolicyStore PersistentStore -Confirm:$false -ErrorAction SilentlyContinue; Set-NetIPInterface -InterfaceIndex $i -AddressFamily IPv4 -Dhcp Enabled -ErrorAction SilentlyContinue }; netsh interface ipv4 set address name=\"$($a.Name)\" static $ip 255.255.255.0 none; Write-Host ('SUKCES: Ustawiono statyczny adres IP ' + $ip) -ForegroundColor Green; if ($a.Status -ne 'Up') { Write-Host 'UWAGA: ta karta nie ma polaczenia - sprawdz kabel do zasilacza PoE.' -ForegroundColor Yellow } } else { Write-Host 'BLAD: Nie wykryto karty Ethernet!' -ForegroundColor Red }"

netsh advfirewall firewall delete rule name="PicoGo_Panel" >nul 2>&1
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=UDP localport=67,68,5005,5006 >nul
netsh advfirewall firewall add rule name="PicoGo_Panel" dir=in action=allow protocol=TCP localport=8000 >nul

echo Odblokowano porty komunikacyjne w Zaporze Windows.

rem Jesli przy pierwszym uruchomieniu panelu zamknieto okno Zapory Windows, system utworzyl
rem regule BLOKUJACA program. Taka regula wygrywa z regulami portow powyzej, wiec ja usuwamy.
powershell -NoProfile -ExecutionPolicy Bypass -Command "$blk = @(Get-NetFirewallRule -Action Block -Direction Inbound -ErrorAction SilentlyContinue | Where-Object { ($_ | Get-NetFirewallApplicationFilter).Program -like '*PicoGoControlPanel.exe' }); if ($blk.Count -gt 0) { $blk | Remove-NetFirewallRule; Write-Host ('Usunieto reguly Zapory blokujace PicoGoControlPanel.exe: ' + $blk.Count) -ForegroundColor Yellow }"

echo.
pause