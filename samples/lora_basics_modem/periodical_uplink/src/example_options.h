/*
 * Copyright (c) 2024 Semtech Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EXAMPLE_OPTIONS_H
#define EXAMPLE_OPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief LoRaWAN User credentials
 */
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x30, 0xb0, 0xd5, 0x20, 0x1c, 0x73, 0x1f, 0xf7 \
    }
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0x27, 0xa2, 0xca, 0x69, 0x43, 0x82, 0x70, 0x8e \
    }
#define USER_LORAWAN_GEN_APP_KEY                                                                       \
    {                                                                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0x85, 0x4a, 0xac, 0x12, 0x28, 0xc8, 0x41, 0xfe, 0xf0, 0xdd, 0xd9, 0x04, 0x9d, 0x9f, 0xc6, 0x65 \
    }

/**
 * @brief Modem Region define
 */
#ifndef MODEM_EXAMPLE_REGION
#if !defined( SX128X )
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_EU_868
#else
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_WW2G4
#endif
#endif  // MODEM_EXAMPLE_REGION
// clang-format on

#ifdef __cplusplus
}
#endif

#endif  // EXAMPLE_OPTIONS_H

/* --- EOF ------------------------------------------------------------------ */
