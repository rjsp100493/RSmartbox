@echo off
rem Arranca cloudflared y la app Flask en ventanas separadas
rem Ajusta rutas si cambian

set BASE=C:\Users\rodri\Documents\PlatformIO\Projects\RSmartbox
set CLOUDFLARED=C:\Users\rodri\cloudflared.exe
set TUNNEL_NAME=rsmartbox

rem Inicia el túnel (requiere config.yml en %USERPROFILE%\.cloudflared)
start "cloudflared" cmd /k "%CLOUDFLARED% tunnel run %TUNNEL_NAME%"

rem Inicia la app (puerto 5000, con recarga porque serve.py está en debug=True)
start "rsmartbox-serve" cmd /k "cd /d %BASE% && python serve.py"

exit /b
