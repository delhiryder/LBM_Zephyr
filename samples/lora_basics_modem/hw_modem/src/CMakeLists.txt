# Copyright (c) 2024 Semtech Corporation - Félix Piédallu
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
target_sources(app PRIVATE main.c modem_pinout.c)
target_sources_ifdef(CONFIG_MCUMGR_TRANSPORT_BT app PRIVATE bluetooth.c)
add_subdirectory(hw_modem)
