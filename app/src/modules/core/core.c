/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(core);

#include <zephyr/zbus/zbus.h>

#include "message_channel.h"

#define CORE_THREAD_STACKSIZE 4096*2
#define CORE_THREAD_PRIORITY 8

ZBUS_SUBSCRIBER_DEFINE(core, MESSAGE_QUEUE_SIZE);

/* 
Extracts fields from the message headers and 
routes the message to the required destinantion.
There are 3 network routes, and 2 request routes.

Network routes:
	1: If TARGET_DEV_TYPE > THIS_DEVTYPE, send uplink
	2: If TARGET_DEV_TYPE < THIS_DEVTYPE, send dnlink
	3: If TARGET_DEV_TYPE == THIS_DEVTYPE, send to consumer 

Request routes - only applicable to case 3 network route,
 i.e. only applicable to the intended recipient device:
	1: If HEADER_TYPE == GET, send to handle_request_get
	2: If HEADER_TYPE == PUT, send to handle_request_put
*/
static void core_thread(void)
{
	LOG_INF("start");
	int ret;

	/* Get this device's dev_id */
	struct dev_id_t this_dev_id;
	get_dev_id(&this_dev_id);

	struct payload_t payload;
	devtype_e target_dev_type;
	topic_e topic;

	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&core, &chan, K_FOREVER)) 
	{				
		if (&PAYLOAD_CHAN == chan) { /* Handle the network routing */
			/* Read the payload */
			zbus_chan_read(&PAYLOAD_CHAN, &payload, K_SECONDS(1));

			LOG_HEXDUMP_DBG(payload.data, 16, "payload");

			/* Check if the message is localhost. 
			Do this by checking the length of the payload. */
			if (payload.len == LOCALHOST_REQUEST_LEN) {
				LOG_INF("origin is localhost");
			}

			/* Read target_dev_type from the payload header */
			target_dev_type = read_target_dev_type_from_header(&payload);

			if (target_dev_type < THIS_DEVTYPE)
			{ /* Send downlink */
				LOG_INF("send downlink");
				ret = zbus_chan_pub(&DOWNLINK_CHAN, &payload, K_SECONDS(1));
				if (ret) {
					LOG_ERR("zbus_chan_pub, error:%d", ret);
					SEND_FATAL_ERROR();
				}
			} 
			else if (target_dev_type > THIS_DEVTYPE)
			{ /* Send uplink */
				LOG_INF("send upwnlink");
				ret = zbus_chan_pub(&UPLINK_CHAN, &payload, K_SECONDS(1));
				if (ret) {
					LOG_ERR("zbus_chan_pub, error:%d", ret);
					SEND_FATAL_ERROR();
				}
			} 
			else {/* Handle the request routing */
				/* Read target_dev_type from the payload header */
				topic = read_topic_from_header(&payload);
				
				/* Dispatch the payload to the appropriate module */
				switch (topic) 
				{
					case TOPIC_SYSTIME:
						ret = zbus_chan_pub(&SYSTIME_CHAN, &payload, K_SECONDS(1));
						if (ret) {
							LOG_ERR("zbus_chan_pub, error:%d", ret);
							SEND_FATAL_ERROR();
						}
						break;
					default:
						LOG_WRN("Received unhandled topic type: %d", topic);
						break;
				}
			}
		}
	}
}

K_THREAD_DEFINE(core_thread_id, CORE_THREAD_STACKSIZE,
		core_thread, NULL, NULL, NULL, CORE_THREAD_PRIORITY, 0, 0);