@echo off
chcp 65001 >nul
cd /d "%~dp0"
title S4Fly 手机共享服务器
echo 正在启动 S4Fly 手机共享服务器...
python 共享服务器.py
echo.
echo 服务器已退出。按任意键关闭窗口。
pause >nul
