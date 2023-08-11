#ifndef _MESSAGE_CHANNEL_H_
#define _MESSAGE_CHANNEL_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>

#include "dev_id.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESSAGE_QUEUE_SIZE 4
#define PAYLOAD_MAX_LEN 4096

#define PAYLOAD_HEADER_LEN (4 + MAX_DEV_ID_LEN + MAX_DEV_ID_LEN)
#define MAX_PAYLOAD_BODY_LEN (PAYLOAD_MAX_LEN - PAYLOAD_HEADER_LEN)
#define LOCALHOST_REQUEST_LEN (4 + MAX_DEV_ID_LEN)
#define ORIGIN_DEV_ID_INDEX 4
#define TARGET_DEV_ID_INDEX (ORIGIN_DEV_ID_INDEX + MAX_DEV_ID_LEN)

#define THIS_DEVTYPE DEVTYPE_GATEWAY_LOWER
#define NEXT_UPLINK_DEVTYPE (THIS_DEVTYPE+1)
#define NEXT_DOWNLINK_DEVTYPE (THIS_DEVTYPE+1)

#define HEXDUMP_DBG_PRINT_LEN 32

typedef enum {
    DEVTYPE_EDGE,
	DEVTYPE_GATEWAY_LOWER,
	DEVTYPE_GATEWAY_UPPER,
	DEVTYPE_SERVER,
} devtype_e;

typedef enum {
    TOPIC_SYSTIME
} topic_e;

typedef enum {
    REQUEST_GET,
    REQUEST_PUT,
    REQUEST_SET
} request_e;

/* Header format is:
<origin_dev_type>, <target_dev_type>, <topic>, <request_type>, <origin_dev_id>, <target_dev_id>, <encoded_message>
1b               , 1b,                 , 1b,    , 1b,           , 16b            , 16b               , varies */
// struct header_t {
// 	uint8_t origin_dev_type;
// 	uint8_t target_dev_type;
// 	uint8_t topic;
// 	uint8_t request_type;
// 	uint8_t origin_dev_id[MAX_DEV_ID_LEN];
// 	uint8_t target_dev_id[MAX_DEV_ID_LEN];
// };

struct payload_t{
	int len;
	uint8_t data[PAYLOAD_MAX_LEN];
};

void prepare_get_response_header(struct payload_t *payload);

void prepare_get_request_header(struct payload_t *payload, topic_e topic);

int encode_response_body(uint8_t *buf, const void *fields, void *msg, size_t len);

int decode_response_body(uint8_t *buf, const void *fields, void *msg, size_t len);

devtype_e read_target_dev_type_from_header(const struct payload_t *payload);

topic_e read_topic_from_header(const struct payload_t *payload);

request_e read_request_type_from_header(const struct payload_t *payload);

/** @brief Macro used to send a message on the FATAL_ERROR_CHANNEL.
 *	   The message will be handled in the error module.
 */
#define SEND_FATAL_ERROR()									\
	int not_used = -1;									\
	if (zbus_chan_pub(&FATAL_ERROR_CHAN, &not_used, K_SECONDS(10))) {			\
		LOG_ERR("Sending a message on the fatal error channel failed, rebooting");	\
		LOG_PANIC();									\
		IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)));					\
	}

ZBUS_CHAN_DECLARE(
	PAYLOAD_CHAN, \
	UPLINK_CHAN, \
	DOWNLINK_CHAN, \
	SYSTIME_CHAN, \
	FATAL_ERROR_CHAN\
);

#ifdef __cplusplus
}
#endif

#endif /* _MESSAGE_CHANNEL_H_ */