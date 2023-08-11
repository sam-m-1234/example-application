#include <string.h>
#include <zephyr/drivers/hwinfo.h>

#include "dev_id.h"

static inline int dev_id_to_str(const uint8_t *data, \
								char *str, size_t len, size_t var_dev_id_len)
{
	snprintk(str, len, \
		"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
		data[0], data[1], data[2], data[3], data[4], data[5], 
		data[6], data[7], data[8], data[9], data[10], data[11], 
		data[12], data[13], data[14], data[15]);

	/* Clip the length of the string according to the actual dev_id length */
	str[var_dev_id_len*2] = '\0';

	return strlen(str);
}

void get_dev_id(struct dev_id_t *dev_id)
{
	dev_id->raw_len = hwinfo_get_device_id(dev_id->raw, sizeof(dev_id->raw));
	dev_id_to_str(dev_id->raw, dev_id->str, sizeof(dev_id->str), dev_id->raw_len);
}