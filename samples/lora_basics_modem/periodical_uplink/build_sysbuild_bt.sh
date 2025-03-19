#!/bin/sh
west build -p -b nrf5340dk/nrf5340/cpuapp --sysbuild -- -DSB_CONFIG_BOOTLOADER_MCUBOOT=y -DEXTRA_CONF_FILE="overlay-bt.conf" -DSHIELD=semtech_sx1262mb2cas

