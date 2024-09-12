/*
 * Copyright (c) 2024 Semtech Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SMTC_HAL_UART_H__
#define __SMTC_HAL_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void hw_modem_uart_dma_start_rx( uint8_t* buff, uint16_t size );
void hw_modem_uart_dma_stop_rx( void );
void hw_modem_uart_tx( uint8_t* buff, uint8_t len );
void hw_modem_async_uart_init( uint8_t *buf, int buf_size, volatile bool *cmd_available );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_UART_H__

/* --- EOF ------------------------------------------------------------------ */
