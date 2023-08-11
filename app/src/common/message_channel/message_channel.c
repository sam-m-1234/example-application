/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(message_channel);

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/drivers/hwinfo.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include "message_channel.h"
#include "dev_id.h"

/* Header format is:
<origin_dev_type>, <target_dev_type>, <topic>, <request_type>, <origin_dev_id>, <target_dev_id>, <encoded_message>
1b               , 1b,                 , 1b,    , 1b,           , 16b            , 16b               , varies */

/* Constructs the new header for the response:
The request origin becomes the responsese target.  
The topic remains the same. 
The request type becomes 'put'.
The request origin_dev_id becomes the response target_dev_id */
void prepare_get_response_header(struct payload_t *payload)
{
	payload->data[0] = payload->data[1];
	payload->data[1] = (uint8_t)THIS_DEVTYPE;
	payload->data[3] = (uint8_t)REQUEST_PUT;
	memcpy(&payload->data[4 + MAX_DEV_ID_LEN], &payload->data[4], MAX_DEV_ID_LEN);
	hwinfo_get_device_id(&payload->data[4], MAX_DEV_ID_LEN);

	payload->len = PAYLOAD_HEADER_LEN;
}

void prepare_get_request_header(struct payload_t *payload, topic_e topic)
{
	/* Add the header */
	payload->data[0] = (uint8_t)THIS_DEVTYPE;
	payload->data[1] = (uint8_t)NEXT_UPLINK_DEVTYPE;
	payload->data[2] = (uint8_t)topic;
	payload->data[3] = (uint8_t)REQUEST_GET;
	hwinfo_get_device_id(&payload->data[4], MAX_DEV_ID_LEN);
	/* Note: No need to set target_dev_id 
	for localhost requests. */
	payload->len = 4 + MAX_DEV_ID_LEN;
}

int encode_response_body(uint8_t *buf, const void *fields, void *msg, size_t len)
{
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buf, len);
	if (!pb_encode(&stream, fields, msg)) {
		LOG_ERR("Encoding message failed: %s", PB_GET_ERROR(&stream));
	}

	return stream.bytes_written;
}

int decode_response_body(uint8_t *buf, const void *fields, void *msg, size_t len)
{
	/* Create a stream that reads from the buffer. */
	pb_istream_t stream = pb_istream_from_buffer(buf, len);
	if (!pb_decode(&stream, fields, msg)) {
		LOG_ERR("Decoding failed: %s\n", PB_GET_ERROR(&stream));
		return -1;
	}

	return 0;
}

devtype_e read_target_dev_type_from_header(const struct payload_t *payload)
{
	return (devtype_e)payload->data[1];
}

topic_e read_topic_from_header(const struct payload_t *payload)
{
	return (topic_e)payload->data[2];
}

request_e read_request_type_from_header(const struct payload_t *payload)
{
	return (request_e)payload->data[3];
}

ZBUS_CHAN_DEFINE(PAYLOAD_CHAN,	/* Name */
		 struct payload_t,				/* Message type */
		 NULL,			        	/* Validator */
		 NULL,				        /* User data */
		 ZBUS_OBSERVERS(core),	/* Observers */
		 ZBUS_MSG_INIT(0)		    /* Initial value {0} */
);

ZBUS_CHAN_DEFINE(UPLINK_CHAN,	/* Name */
		 struct payload_t,				/* Message type */
		 NULL,			        	/* Validator */
		 NULL,				        /* User data */
		 ZBUS_OBSERVERS(core, uplink),	/* Observers */
		 ZBUS_MSG_INIT(0)		    /* Initial value {0} */
);

ZBUS_CHAN_DEFINE(DOWNLINK_CHAN,	/* Name */
		 struct payload_t,				/* Message type */
		 NULL,			        	/* Validator */
		 NULL,				        /* User data */
		 ZBUS_OBSERVERS(core, downlink),	/* Observers */
		 ZBUS_MSG_INIT(0)		    /* Initial value {0} */
);

ZBUS_CHAN_DEFINE(SYSTIME_CHAN,	/* Name */
		 struct payload_t,				/* Message type */
		 NULL,			        	/* Validator */
		 NULL,				        /* User data */
		 ZBUS_OBSERVERS(core, systime),	/* Observers */
		 ZBUS_MSG_INIT(0)		    /* Initial value {0} */
);

ZBUS_CHAN_DEFINE(FATAL_ERROR_CHAN,
		 int,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(error),
		 ZBUS_MSG_INIT(0)
);