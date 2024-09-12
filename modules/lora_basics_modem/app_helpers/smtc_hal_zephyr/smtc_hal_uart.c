/*
 * Copyright (c) 2024 Semtech Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "smtc_hal_uart.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hal_uart, 3);

static const struct device *prv_uart_dev = DEVICE_DT_GET(DT_ALIAS(smtc_hal_uart));

static uint8_t *prv_buff;
static uint16_t prv_i;
static uint16_t prv_size;

/**
 * @brief Interrupt driven UART handler
 *
 * @param[in] dev UART device pointer
 * @param[in] user_data not used
 */
static void prv_uart_irq_rx_callback_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t c;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	int status = uart_err_check(dev);
	if (status > 0) {
		LOG_ERR("UART error detected: %d", status);
	}

	/* read until FIFO empty */
	while (uart_fifo_read(dev, &c, 1) == 1) {
		// __ASSERT(prv_i < prv_size, "Received more data than the buffer can hold!");
		prv_buff[prv_i++] = c;
	}
}

/**
 * @brief Configure UART in interrupt based mode
 *
 * Data must be received into the buffer provided
 *
 * @param[in] buff The buffer to store the received data in
 * @param[in] size The size of the buffer
 */
void hw_modem_uart_dma_start_rx(uint8_t *buff, uint16_t size)
{
	/* Remember where to store data into */
	int ret = 0;
	prv_i = 0;
	prv_buff = buff;
	prv_size = size;

	LOG_INF("prv_i: %d", prv_i);
	LOG_INF("prv_buff[0]: %x", prv_buff[0]);

	/* configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(prv_uart_dev,
		prv_uart_irq_rx_callback_handler, NULL);
	// printk("uart_irq_callback_user_data_set returned %d\n", ret);
	uart_irq_rx_enable(prv_uart_dev);
	LOG_WRN("UART RX started");
}

/**
 * @brief Turn off the UART RX
 */
void hw_modem_uart_dma_stop_rx(void)
{
	uart_irq_rx_disable(prv_uart_dev);
	LOG_WRN("UART RX stopped");

	LOG_INF("prv_i: %d", prv_i);
	for (int i=0; i<prv_i; i++) {
		LOG_INF("prv_buff[%d]: %x", i, prv_buff[i]);
	}
}

/**
 * @brief Send data over UART, polling
 *
 * @param[in] buff The buffer to send
 * @param[in] len The length of the buffer
 */
void hw_modem_uart_tx(uint8_t *buff, uint8_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(prv_uart_dev, buff[i]);
	}
}

#define BUF_SIZE 256
static K_MEM_SLAB_DEFINE(uart_slab, BUF_SIZE, 3, 4);

static uint8_t *async_buf;

typedef struct {
  const struct device *uart_dev;
  uint8_t *rx_buf;
  int rx_buf_size;
  volatile bool *modem_cmd_available;
} uart_async_user_data_t;

static void uart_callback(const struct device *dev, struct uart_event *evt,
                          void *user_data) {
  uart_async_user_data_t *user_data_instance = user_data;

  const struct device *uart = user_data_instance->uart_dev;
  int err;

  switch (evt->type) {
  case UART_TX_DONE:
    LOG_INF("Tx sent %d bytes", evt->data.tx.len);
    break;

  case UART_TX_ABORTED:
    LOG_ERR("Tx aborted");
    break;

  case UART_RX_RDY: {
    LOG_INF("offset: %d, buf[0]: %x", evt->data.rx.offset, evt->data.rx.buf[0]);

    memset(user_data_instance->rx_buf, 0xFF, user_data_instance->rx_buf_size);
    memcpy(user_data_instance->rx_buf, evt->data.rx.buf + evt->data.rx.offset,
           evt->data.rx.len);
    *(user_data_instance->modem_cmd_available) = true;

    LOG_INF("Received data %d bytes", evt->data.rx.len);
    break;
  }

  case UART_RX_BUF_REQUEST: {
    uint8_t *buf;

    err = k_mem_slab_alloc(&uart_slab, (void **)&buf, K_NO_WAIT);
    __ASSERT(err == 0, "Failed to allocate slab");

    err = uart_rx_buf_rsp(uart, buf, BUF_SIZE);
    __ASSERT(err == 0, "Failed to provide new buffer");
    break;
  }

  case UART_RX_BUF_RELEASED:
    k_mem_slab_free(&uart_slab, (void *)evt->data.rx_buf.buf);
    break;

  case UART_RX_DISABLED:
    break;

  case UART_RX_STOPPED:
    break;
  }
}

static uart_async_user_data_t user_data_1;

void hw_modem_async_uart_init(uint8_t *buf, int buf_size,
                              volatile bool *cmd_available) {
  int err;

  user_data_1.uart_dev = prv_uart_dev;
  user_data_1.rx_buf = buf;
  user_data_1.rx_buf_size = buf_size;
  user_data_1.modem_cmd_available = cmd_available;

  err = k_mem_slab_alloc(&uart_slab, (void **)&async_buf, K_NO_WAIT);
  __ASSERT(err == 0, "Failed to alloc slab");

  err = uart_callback_set(prv_uart_dev, uart_callback, (void *)&user_data_1);
  LOG_INF("error code: %d\n", err);
  __ASSERT(err == 0, "Failed to set callback");

  err = uart_rx_enable(prv_uart_dev, async_buf, BUF_SIZE, 10000);
  __ASSERT(err == 0, "Failed to enable RX");
}

