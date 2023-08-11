#ifndef DEV_ID_H_
#define DEV_ID_H_

#ifdef __cplusplus
extern "C" {
#endif

// example dev_id string: 66127FF350BCB6ED
#define MAX_DEV_ID_LEN 8
#define MAX_DEV_ID_STR_LEN 17 // add 1 extra for null byte

struct dev_id_t {
	uint8_t raw_len;
	uint8_t raw[MAX_DEV_ID_LEN];
	char str[MAX_DEV_ID_STR_LEN];
};

void get_dev_id(struct dev_id_t *dev_id);

#ifdef __cplusplus
}
#endif

#endif /* DEV_ID_H_ */
