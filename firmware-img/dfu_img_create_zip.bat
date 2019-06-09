@echo off

%SET APP_VERSION=1

nrfutil keys display --key pk --format code private.pem
nrfutil pkg generate --hw-version 52 --sd-req 0xAE --application-version 1 --application ..\Project-nRF52840\Output\Debug\Exe\BLE-nRF52840.hex --key-file private.pem ble_cli_dfu_1.zip