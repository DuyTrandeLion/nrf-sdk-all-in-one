@echo off

set /p bootloader_version="Enter bootloader version: "
set /p app_version="Enter application version: "
set /p hex_file=".hex file name: "

nrfutil settings generate --family NRF52840 --application %hex_file% --application-version %app_version% --bootloader-version %bootloader_version% --bl-settings-version 2 dfu_settings_%app_version%.hex

pause