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

//        0x9e, 0xef, 0x3e, 0xbb, 0x6a, 0x79, 0x1a, 0x18 (Class C)

#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x04, 0x87, 0x24, 0x2c, 0x62, 0xfa, 0xcf, 0xac \
    }

    //        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 (Class C)

#define USER_LORAWAN_JOIN_EUI                          \
    {                                                 \
        0xb7, 0x48, 0xbc, 0xf8, 0x69, 0x44, 0xf4, 0xc1 \
    }
#define USER_LORAWAN_GEN_APP_KEY                                                                       \
    {                                                                                                  \
        0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }

//        0x1e, 0x8c, 0x22, 0xf2, 0x37, 0x45, 0x92, 0xa6, 0xc0, 0x83, 0x18, 0x08, 0xe3, 0x39, 0x12, 0x54 (Class C)

#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0x23, 0x15, 0xc6, 0x97, 0x36, 0x7d, 0xe8, 0x51, 0xb0, 0xf7, 0x85, 0x8d, 0xb0, 0x43, 0x23, 0x0a \
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
