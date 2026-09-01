#ifndef AXDR_ZEPHYR_SYS_CRC_H_
#define AXDR_ZEPHYR_SYS_CRC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/compat.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t crc8_ccitt(uint8_t val, const void *buf, size_t cnt);
uint32_t crc32_ieee(const uint8_t *data, size_t len);
uint32_t crc32_ieee_update(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AXDR_ZEPHYR_SYS_CRC_H_ */
