/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_WRN
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_slave);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/zbus/zbus.h>

#include "message_channel.h"

#define SPI_ASYNC_CALL_CB_THREAD_STACKSIZE 4096*4
#define SPI_SLAVE_TX_THREAD_STACKSIZE 4096*4
#define SPI_SLAVE_TX_THREAD_PRIORITY 8

/* Register subscriber */
ZBUS_SUBSCRIBER_DEFINE(uplink, MESSAGE_QUEUE_SIZE);

static struct gpio_dt_spec sig_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sig0), gpios, {0});

#define MY_SPI_SLAVE  DT_NODELABEL(my_spi_slave)
static const struct device *spi_slave_dev;

#define SIG_WRITE_FLAG 0xFFFF
static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
static struct k_poll_event async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &async_sig);
K_THREAD_STACK_DEFINE(spi_async_stack, SPI_ASYNC_CALL_CB_THREAD_STACKSIZE);

static uint8_t tx_buf[PAYLOAD_MAX_LEN];
static uint8_t rx_buf[PAYLOAD_MAX_LEN];

static const struct spi_buf s_tx_buf[] = {
	{
		.buf = tx_buf,
		.len = sizeof(tx_buf)
	}
};

static struct spi_buf s_rx_buf[] = {
	{
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	}
};

static const struct spi_buf_set s_tx = {
	.buffers = (const struct spi_buf *)&s_tx_buf,
	.count = ARRAY_SIZE(s_tx_buf)
};

static const struct spi_buf_set s_rx = {
	.buffers = (const struct spi_buf *)&s_rx_buf,
	.count = ARRAY_SIZE(s_rx_buf)
};

static const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE,
};

static void spi_slave_init(void)
{
	spi_slave_dev = DEVICE_DT_GET(MY_SPI_SLAVE);
	if(!device_is_ready(spi_slave_dev)) {
		printk("SPI slave device not ready!\n");
	}
}

static void init_spi_slave_interrupt(void)
{
	int ret;
	if (sig_gpio.port && !device_is_ready(sig_gpio.port)) {
		LOG_ERR("Error: sig_gpio device %s is not ready; ignoring it",
		       sig_gpio.port->name);
		sig_gpio.port = NULL;
	}
	if (sig_gpio.port) {
		ret = gpio_pin_configure_dt(&sig_gpio, GPIO_OUTPUT);
		if (ret != 0) {
			LOG_ERR("Error %d: failed to configure LED device %s pin %d",
			       ret, sig_gpio.port->name, sig_gpio.pin);
			sig_gpio.port = NULL;
		} else {
			LOG_INF("Set up sig_gpio at %s pin %d", sig_gpio.port->name, sig_gpio.pin);
		}
	}

	gpio_pin_set_dt(&sig_gpio, GPIO_OUTPUT_INACTIVE);
}

static inline void trigger_spi_slave_interrupt()
{
	gpio_pin_set_dt(&sig_gpio, 1);
	gpio_pin_set_dt(&sig_gpio, 0);
}

static void spi_async_call_cb(struct k_poll_event *async_evt,
			      void *, void *)
{
	int ret;
	int result;

	struct payload_t payload;

	LOG_DBG("spi_async_call_cb thread started");

	while (1) {
		ret = k_poll(async_evt, 1, K_FOREVER);
		result = async_evt->signal->result;
		LOG_DBG("rx_thread got event: %d", result);

		if (result != SIG_WRITE_FLAG){
			int ret = spi_transceive_signal(spi_slave_dev, &spi_slave_cfg, &s_tx, &s_rx, &async_sig);
			if(ret != 0){
				LOG_ERR("SPI slave read error: %i", ret);
				return;
			}

			/* Copy rx_buf into msg */
			memcpy(payload.data, rx_buf, PAYLOAD_MAX_LEN);
			payload.len = PAYLOAD_MAX_LEN;

			ret = zbus_chan_pub(&UPLINK_CHAN, &payload, K_SECONDS(1));
			if (ret) {
				LOG_ERR("zbus_chan_pub, error:%d", ret);
				SEND_FATAL_ERROR();
				return;
			}
		}
		else {
			int ret = spi_write_signal(spi_slave_dev, &spi_slave_cfg, &s_tx, &async_sig);
			if(ret != 0){
				LOG_ERR("SPI slave read error: %i", ret);
				return;
			}
		}

		/* Reinitializing for next call */
		async_evt->signal->signaled = 0U;
		async_evt->state = K_POLL_STATE_NOT_READY;
	}
}

static void spi_slave_tx_thread(void)
{
	int ret;

	struct payload_t payload;

	struct k_thread async_thread;
	k_tid_t async_thread_id;
	async_thread_id = k_thread_create(&async_thread,
					  spi_async_stack, \
					  SPI_ASYNC_CALL_CB_THREAD_STACKSIZE,
					  (k_thread_entry_t)spi_async_call_cb,
					  &async_evt, NULL, NULL,
					  K_PRIO_COOP(7), 0, K_NO_WAIT);

	init_spi_slave_interrupt();
	spi_slave_init();
	LOG_INF("SPI slave started");

	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&uplink, &chan, K_FOREVER)) {
		if (&UPLINK_CHAN == chan) {

			ret = zbus_chan_read(&UPLINK_CHAN, &payload, K_SECONDS(1));
			if (ret) {
				LOG_ERR("zbus_chan_read, error: %d", ret);
				SEND_FATAL_ERROR();
				return;
			}

			/* Copy rx_buf into msg */
			memcpy(tx_buf, payload.data, payload.len);

			LOG_DBG("Slave TX transfer start");
			trigger_spi_slave_interrupt();

			k_poll_signal_raise(&async_sig, SIG_WRITE_FLAG);
		}
	}
}

K_THREAD_DEFINE(spi_slave_tx_thread_id, SPI_SLAVE_TX_THREAD_STACKSIZE,
		spi_slave_tx_thread, NULL, NULL, NULL, SPI_SLAVE_TX_THREAD_PRIORITY, 0, 0);


// /*
//  * Copyright (c) 2016 Intel Corporation
//  *
//  * SPDX-License-Identifier: Apache-2.0
//  */

// #define LOG_LEVEL LOG_LEVEL_WRN
// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(uplink);

// #include <zephyr/kernel.h>

// #include <zephyr/zbus/zbus.h>

// #include "message_channel.h"

// #define UPLINK_RX_THREAD_STACKSIZE 4096*2
// #define UPLINK_TX_THREAD_STACKSIZE 4096*2

// #define UPLINK_RX_THREAD_PRIORITY 8
// #define UPLINK_TX_THREAD_PRIORITY 8

// /* Register subscriber */
// ZBUS_SUBSCRIBER_DEFINE(uplink, MESSAGE_QUEUE_SIZE);

// K_SEM_DEFINE(uplink_rx_sem, 0, 1); // TODO Use msgq instead

// static void uplink_rx_thread(struct k_poll_event *async_evt,
// 			      struct k_sem *caller_sem, void *unused)
// {
// 	struct payload_t payload;
// 	int ret;

// 	while(1) {	
// 		k_sem_take(&uplink_rx_sem, K_FOREVER); // TODO Use msgq instead

// 		/* Copy rx_buf into msg */
// 		// memcpy(rx_msg.data, rx_buf, PAYLOAD_MAX_LEN);
// 		// rx_msg.len = PAYLOAD_MAX_LEN;

// 		ret = zbus_chan_pub(&PAYLOAD_CHAN, &payload, K_SECONDS(1));
// 		if (ret) {
// 			LOG_ERR("zbus_chan_pub, error:%d", ret);
// 			SEND_FATAL_ERROR();
// 		}
// 	}
// }

// K_THREAD_DEFINE(uplink_rx_thread_id, UPLINK_RX_THREAD_STACKSIZE,
// 		uplink_rx_thread, NULL, NULL, NULL, UPLINK_RX_THREAD_PRIORITY, 0, 0);

// static void uplink_tx_thread(void)
// {
// 	struct payload_t payload;
// 	int ret;
// 	const struct zbus_channel *chan;

// 	while (!zbus_sub_wait(&uplink, &chan, K_FOREVER)) {
// 		if (&UPLINK_CHAN == chan) {

// 			ret = zbus_chan_read(&UPLINK_CHAN, &payload, K_SECONDS(1));
// 			if (ret) {
// 				LOG_ERR("zbus_chan_read, error: %d", ret);
// 				SEND_FATAL_ERROR();
// 				return;
// 			}

// 			/* Copy rx_buf into msg */
// 			// memcpy(tx_buf, tx_msg.data, tx_msg.len);

// 			/* Send over BT */
// 			// TODO
// 		}
// 	}
// }

// K_THREAD_DEFINE(uplink_tx_thread_id, UPLINK_TX_THREAD_STACKSIZE,
// 		uplink_tx_thread, NULL, NULL, NULL, UPLINK_TX_THREAD_PRIORITY, 0, 0);