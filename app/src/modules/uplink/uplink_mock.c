/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uplink);

#include <zephyr/kernel.h>

#include <zephyr/zbus/zbus.h>

#include "message_channel.h"

#define UPLINK_RX_THREAD_STACKSIZE 4096*2
#define UPLINK_TX_THREAD_STACKSIZE 4096*2

#define UPLINK_RX_THREAD_PRIORITY 8
#define UPLINK_TX_THREAD_PRIORITY 8

/* Register subscriber */
ZBUS_SUBSCRIBER_DEFINE(uplink, MESSAGE_QUEUE_SIZE);

K_SEM_DEFINE(uplink_rx_sem, 0, 1); // TODO Use msgq instead

static void uplink_rx_thread(struct k_poll_event *async_evt,
			      struct k_sem *caller_sem, void *unused)
{
	LOG_INF("starting uplink_mock rx thread.");
	
	struct payload_t payload;
	int ret;

	while(1) {	
		k_sem_take(&uplink_rx_sem, K_FOREVER); // TODO Use msgq instead

		/* Copy rx_buf into msg */
		// memcpy(rx_msg.data, rx_buf, PAYLOAD_MAX_LEN);
		// rx_msg.len = PAYLOAD_MAX_LEN;

		ret = zbus_chan_pub(&PAYLOAD_CHAN, &payload, K_SECONDS(1));
		if (ret) {
			LOG_ERR("zbus_chan_pub, error:%d", ret);
			SEND_FATAL_ERROR();
		}
	}
}

K_THREAD_DEFINE(uplink_rx_thread_id, UPLINK_RX_THREAD_STACKSIZE,
		uplink_rx_thread, NULL, NULL, NULL, UPLINK_RX_THREAD_PRIORITY, 0, 0);

static void uplink_tx_thread(void)
{
	LOG_INF("starting uplink_mock tx thread.");

	struct payload_t payload;
	int ret;
	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&uplink, &chan, K_FOREVER)) {
		if (&UPLINK_CHAN == chan) {
			LOG_INF("GOT MESSAGE");

			ret = zbus_chan_read(&UPLINK_CHAN, &payload, K_SECONDS(1));
			if (ret) {
				LOG_ERR("zbus_chan_read, error: %d", ret);
				SEND_FATAL_ERROR();
				return;
			}

			/* Copy rx_buf into msg */
			// memcpy(tx_buf, tx_msg.data, tx_msg.len);

			/* Send over BT */
			// TODO
		}
	}
}

K_THREAD_DEFINE(uplink_tx_thread_id, UPLINK_TX_THREAD_STACKSIZE,
		uplink_tx_thread, NULL, NULL, NULL, UPLINK_TX_THREAD_PRIORITY, 0, 0);