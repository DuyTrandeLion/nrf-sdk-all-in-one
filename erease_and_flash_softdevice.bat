@echo off

set SOFTDEVICE=..\..\library\nRF5_SDK_1500\components\softdevice\s140\hex\s140_nrf52_6.0.0_softdevice.hex
nrfjprog --program %SOFTDEVICE% --verify --chiperase -f NRF52
