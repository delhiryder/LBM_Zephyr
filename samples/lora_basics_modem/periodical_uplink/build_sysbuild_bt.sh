#!/bin/sh
west build -p -b nrf5340dk/nrf5340/cpuapp --sysbuild -- -DEXTRA_CONF_FILE="overlay-bt.conf" -DSHIELD=semtech_sx1262mb2cas

