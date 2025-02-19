#!/bin/sh
west build -p -b nrf5340dk/nrf5340/cpuapp -- -DEXTRA_CONF_FILE="overlay-cdc.conf" -DDTC_OVERLAY_FILE="usb.overlay boards/nrf5340dk_nrf5340_cpuapp.overlay" -DSHIELD=semtech_sx1262mb2cas
