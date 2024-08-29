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
        0xAE, 0xFD, 0xC0, 0x05, 0x56, 0xD8, 0xD3, 0x21 \
    }
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0xDE, 0x24, 0x01, 0x38, 0xA2, 0xA0, 0x14, 0x05 \
    }
#define USER_LORAWAN_GEN_APP_KEY                                                                       \
    {                                                                                                  \
        0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0xF7, 0xBD, 0xD2, 0x9D, 0x93, 0xB4, 0x2C, 0xED, 0x36, 0xD8, 0x27, 0x99, 0x72, 0x8D, 0x66, 0x3F \
    }

/**
 * @brief Modem Region define
 */
#ifndef MODEM_EXAMPLE_REGION
#if !defined( SX128X )
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_US_915
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
