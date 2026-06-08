@echo off
REM burn.bat - 雙擊不能用,要從 cmd 執行
REM 用法:
REM   burn Voice_Test            (自動偵測 COM)
REM   burn Voice_Test COM7       (指定 COM)
REM   burn Drone_FC_Full COM8

powershell -ExecutionPolicy Bypass -File "%~dp0burn.ps1" %*
