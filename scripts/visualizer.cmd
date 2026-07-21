@echo off
rem Thin shim so a bare `visualizer` on Windows (cmd.exe or PowerShell, both resolve .cmd via
rem PATHEXT) runs the real logic in visualizer.ps1, next to this file.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0visualizer.ps1" %*
exit /b %ERRORLEVEL%
