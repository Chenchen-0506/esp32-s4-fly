@echo off
chcp 65001 >nul
cd /d "%~dp0"
title S4Fly Share Server
echo.
echo  ============================================
echo   S4Fly Share Server starting...
echo.
echo   Phone (same WiFi):  open http://(PC-IP):8787
echo   Local test:         open http://localhost:8787
echo   Press Ctrl+C to stop.
echo  ============================================
echo.
python share_server.py
echo.
echo Server exited. Press any key to close.
pause >nul
