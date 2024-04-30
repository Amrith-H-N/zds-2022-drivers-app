/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensor_sensornode

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>

#include <app/drivers/sensor/sensornode.h>

LOG_MODULE_REGISTER(sensornode, CONFIG_SENSOR_LOG_LEVEL);

/* packet buffer size, RX queue size */
#define PKT_BUF_SIZE  32U
#define RX_QUEUE_SIZE 2U

/* commands */
#define CMD_EMPTY      0x0DU
#define CMD_AUTOENROLL 0x31U
#define CMD_AUTOID     0x32U

/* packet field positions */
#define PKT_HEADER_POS	      0U
#define PKT_ADDR_POS	      2U
#define PKT_ID_POS	      6U
#define PKT_LEN_POS	      7U
#define PKT_CMD_POS	      9U
#define PKT_CONFIRM_POS	      9U
#define PKT_DATA_POS	      10U
#define PKT_SUM_POS(data_len) (PKT_DATA_POS + (data_len))

/* packet field values */
#define PKT_HEADER_VAL	      0xEF01U
#define PKT_LEN_VAL(data_len) (3U + (data_len))

/* packet total length */
#define PKT_LEN(data_len) (12U + (data_len))

/* packet checksum start position & number of bytes */
#define PKT_SUM_START	      PKT_ID_POS
#define PKT_SUM_CNT(data_len) (4U + (data_len))

/* packet IDs */
#define PKT_ID_CMD 0x01U

/* result codes */
#define RESULT_OK 0x00U

/* autoidentify sequence field positions & step IDs */
#define AUTOID_DATA_LEN	    5U
#define AUTOID_POS_STEP	    0U
#define AUTOID_POS_ID	    1U
#define AUTOID_POS_SCORE    3U
#define AUTOID_STEP_RESULTS 0x05U

/* autoenroll sequence field positions & step IDs */
#define AUTOENROLL_TX_DATA_LEN	5U
#define AUTOENROLL_RX_DATA_LEN	2U
#define AUTOENROLL_POS_ID	0U
#define AUTOENROLL_POS_FREQ	2U
#define AUTOENROLL_POS_STEP	0U
#define AUTOENROLL_STEP_STORAGE 6U

static const struct uart_config uart_config = {
	.baudrate = 57600U,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_2,
};


/**
 * @brief UART RX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_rx_handler(const struct device *dev)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	int n;

	n = uart_fifo_read(config->uart, data->buf_ptr,
			   PKT_BUF_SIZE - data->buf_ctr);
	data->buf_ctr += (uint8_t)n;
	data->buf_ptr += (uint8_t)n;

	if (data->buf_ctr >= PKT_LEN(data->rx_data_len)) {
		LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "RX");
		if (k_msgq_put(&data->rx_queue, data->buf, K_NO_WAIT) < 0) {
			LOG_ERR("RX queue full, dropping packet");
		}
		data->buf_ctr = 0U;
		data->buf_ptr = data->buf;
	}
}

/**
 * @brief UART TX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_tx_handler(const struct device *dev)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	int n;

	if (data->buf_ctr > 0U) {
		n = uart_fifo_fill(config->uart, data->buf_ptr, data->buf_ctr);
		data->buf_ctr -= (uint8_t)n;
		data->buf_ptr += (uint8_t)n;
		return;
	}

	if (uart_irq_tx_complete(config->uart) > 0) {
		data->buf_ptr = data->buf;
		uart_irq_tx_disable(config->uart);
		uart_irq_rx_enable(config->uart);
	}
}

/**
 * @brief UART IRQ handler.
 *
 * @param dev UART device instance.
 * @param user_data User data (sensor instance).
 */
static void uart_cb_handler(const struct device *dev, void *user_data)
{
	const struct device *sensor = user_data;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		if (uart_irq_rx_ready(dev)) {
			uart_cb_rx_handler(sensor);
		}

		if (uart_irq_tx_ready(dev)) {
			uart_cb_tx_handler(sensor);
		}
	}
}

