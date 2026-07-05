@echo off
REM Double-click to run the PowerShell push script
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0push_docs.ps1"
pause
