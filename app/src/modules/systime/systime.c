/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(systime);

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
// #include <date_time.h>
#include <time.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>

#include "message_channel.h"
#include "protobuf/time.pb.h"

#define SYSTIME_THREAD_STACKSIZE 4096*4
#define SYSTIME_THREAD_PRIORITY 8

#define THIS_TOPIC TOPIC_SYSTIME

ZBUS_SUBSCRIBER_DEFINE(systime, MESSAGE_QUEUE_SIZE);

void date_time_core_store(int64_t curr_time_ms)
{
	struct timespec tp = { 0 };
	struct tm ltm = { 0 };
	int ret;

	tp.tv_sec = curr_time_ms / 1000;
	tp.tv_nsec = (curr_time_ms % 1000) * 1000000;

	ret = clock_settime(CLOCK_REALTIME, &tp);
	if (ret != 0) {
		LOG_ERR("Could not set system time, %d", ret);
		return;
	}
	gmtime_r(&tp.tv_sec, &ltm);
	LOG_DBG("System time updated: %04u-%02u-%02u %02u:%02u:%02u",
		ltm.tm_year + 1900, ltm.tm_mon + 1, ltm.tm_mday,
		ltm.tm_hour, ltm.tm_min, ltm.tm_sec);
}

int date_time_set(const struct tm *new_date_time)
{
	int err = 0;
	int64_t date_time_ms;

	if (new_date_time == NULL) {
		LOG_ERR("The passed in pointer cannot be NULL");
		return -EINVAL;
	}

	/** Seconds after the minute. tm_sec is generally 0-59.
	 *  The extra range is to accommodate for leap seconds
	 *  in certain systems.
	 */
	if (new_date_time->tm_sec < 0 || new_date_time->tm_sec > 61) {
		LOG_ERR("Seconds in time structure not in correct format");
		err = -EINVAL;
	}

	/** Minutes after the hour. */
	if (new_date_time->tm_min < 0 || new_date_time->tm_min > 59) {
		LOG_ERR("Minutes in time structure not in correct format");
		err = -EINVAL;
	}

	/** Hours since midnight. */
	if (new_date_time->tm_hour < 0 || new_date_time->tm_hour > 23) {
		LOG_ERR("Hours in time structure not in correct format");
		err = -EINVAL;
	}

	/** Day of the month. */
	if (new_date_time->tm_mday < 1 || new_date_time->tm_mday > 31) {
		LOG_ERR("Day in time structure not in correct format");
		err = -EINVAL;
	}

	/** Months since January. */
	if (new_date_time->tm_mon < 0 || new_date_time->tm_mon > 11) {
		LOG_ERR("Month in time structure not in correct format");
		err = -EINVAL;
	}

	/** Years since 1900. 115 corresponds to the year 2015. */
	if (new_date_time->tm_year < 115 || new_date_time->tm_year > 1900) {
		LOG_ERR("Year in time structure not in correct format");
		err = -EINVAL;
	}

	if (err) {
		return err;
	}

	date_time_ms = (int64_t)timeutil_timegm64(new_date_time) * 1000;

	date_time_core_store(date_time_ms);

	return 0;
}

// https://www.epochconverter.com/programming/c
void print_datetime(void)
{
    time_t     now;
    struct tm  ts;
    char       buf[80];

    /* Get current time */
    time(&now);

    /* Format time, "ddd yyyy-mm-dd hh:mm:ss zzz" */
    ts = *localtime(&now);
    strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    LOG_INF("%s", buf);
}

static void serve_get_request(struct payload_t *payload)
{
	prepare_get_response_header(payload);

	Time content = Time_init_zero;

	time_t now;
	struct tm  ts;

	/* Get the system time */
	time(&now);
	ts = *localtime(&now);

	/* Write vals to content struct */ 
	content.sec = ts.tm_sec;
	content.min = ts.tm_min;
	content.hour = ts.tm_hour;
	content.mday = ts.tm_mday;
	content.mon = ts.tm_mon;
	content.year = ts.tm_year;
	content.wday = ts.tm_wday;

	/* Encode the response body, containing the system time */
	payload->len += encode_response_body(payload->data, \
		(void*)Time_fields, &content, MAX_PAYLOAD_BODY_LEN);

	/* Publish the message */
	int ret = zbus_chan_pub(&PAYLOAD_CHAN, payload, K_SECONDS(1));
	if (ret) {
		LOG_ERR("zbus_chan_pub, error:%d", ret);
		SEND_FATAL_ERROR();
	}
}

static void handle_put(struct payload_t *payload)
{
	Time time_msg = Time_init_zero;
	int ret;

	/* Decode the message */
	ret = decode_response_body(&payload->data[PAYLOAD_HEADER_LEN], \
		Time_fields, (void*)&time_msg, Time_size);

	if (ret != 0){
		LOG_WRN("Error decoding time message");
		return;
	}
	
	/* Set the system time using the decoded response */
	struct tm time = {
		.tm_sec = time_msg.sec,
		.tm_min = time_msg.min,
		.tm_hour = time_msg.hour,
		.tm_mday = time_msg.mday,
		.tm_mon = time_msg.mon,
		.tm_year = time_msg.year,
		.tm_wday = time_msg.wday,
	};

	if(date_time_set(&time) != 0){
		LOG_WRN("Error setting time");
	} else {
		print_datetime();
	}
}

void send_get_request_work_handler(struct k_work *work)
{
	/* Allocate the memory on the heap */
	struct payload_t *payload;
	payload = k_malloc(sizeof(struct payload_t));

	if (payload == NULL) {
		LOG_WRN("Memory not allocated. \
			Try increasing CONFIG_HEAP_MEM_POOL_SIZE.");
		return;
	}

	/* Initialize the buffer */
	memset(payload, 0, sizeof(struct payload_t));

	/* Prepare the header */
	prepare_get_request_header(payload, THIS_TOPIC);
	
	/* Publish the message */
	int ret = zbus_chan_pub(&PAYLOAD_CHAN, payload, K_SECONDS(1));
	if (ret) {
		LOG_ERR("zbus_chan_pub, error:%d", ret);
		k_free(payload);
		SEND_FATAL_ERROR();
	}

	/* Free the allocated memory */
	k_free(payload);
}

K_WORK_DEFINE(send_get_request_work, send_get_request_work_handler);

void send_get_request_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&send_get_request_work);
}

K_TIMER_DEFINE(send_get_request_timer, send_get_request_timer_handler, NULL);

static void systime_thread(void)
{
	struct payload_t payload;
	request_e request_type;

	/* Launch timer to periodically update systime via get request */
	k_timer_start(&send_get_request_timer, K_SECONDS(5), K_SECONDS(10));

	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&systime, &chan, K_FOREVER)) 
	{
		LOG_INF("got payload");
		if (&SYSTIME_CHAN == chan) { 
			/* Read the payload */
			zbus_chan_read(&SYSTIME_CHAN, &payload, K_SECONDS(1));

			/* Read the request type from the payload header */
			request_type = read_request_type_from_header(&payload);

			/* Dispatch request to the appropriate handler */
			switch (request_type){
				case REQUEST_GET:
					serve_get_request(&payload);
					break;
				case REQUEST_PUT:
					handle_put(&payload);
					break;
				default:
					LOG_WRN("Received unhandled request type: %d",\
						request_type);
					break;
			}
		}
	}
}

K_THREAD_DEFINE(systime_thread_id, SYSTIME_THREAD_STACKSIZE,
		systime_thread, NULL, NULL, NULL, SYSTIME_THREAD_PRIORITY, 0, 0);