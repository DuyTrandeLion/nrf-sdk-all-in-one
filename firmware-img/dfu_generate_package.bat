@echo off

set /p app_version="Enter application version: "
set /p hex_file=".hex file name: "

nrfutil keys display --key pk --format code private.pem
nrfutil pkg generate --hw-version 52 --sd-req 0xCA --application-version %app_version% --application %hex_file% --key-file private.pem ble_dfu_pkg_%app_version%.zip