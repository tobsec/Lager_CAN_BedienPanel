@echo off
rem https://github.com/tobsec/mcp-can-boot-flash-app

rem Need to have NodeJS

rem Maybe need this for https://github.com/tobsec/cs-pcan-usb
rem $ choco install python visualstudio2022-workload-vctools -y

set CURRENT_DIR=%~dp0
set FIRMWARE=%CURRENT_DIR%.pio\build\m328p-CAN\firmware.hex

rem @echo on

call npx C:\data\mcp-can-boot-flash-app -f %FIRMWARE% -p m328p -m 0x000B --bitrate 125 --reset 016#A0 --can-id-mcu 0x1F1 --can-id-remote 0x1F2 --sff

pause