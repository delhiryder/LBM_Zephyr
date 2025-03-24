# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_LORA_BASICS_MODEM lorawan_lbm)
add_subdirectory_ifdef(CONFIG_MCUMGR_TRANSPORT_LBM smp_lbm)