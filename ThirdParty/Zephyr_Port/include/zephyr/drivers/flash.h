#ifndef AXDR_ZEPHYR_DRIVERS_FLASH_H_
#define AXDR_ZEPHYR_DRIVERS_FLASH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <zephyr/compat.h>

#ifdef __cplusplus
extern "C" {
#endif

struct device;

struct flash_parameters {
    size_t write_block_size;
    struct {
        bool no_explicit_erase : 1;
    } caps;
    uint8_t erase_value;
};

struct flash_pages_info {
    off_t start_offset;
    size_t size;
    uint32_t index;
};

int flash_read(const struct device *dev, off_t offset, void *data, size_t len);
int flash_write(const struct device *dev, off_t offset, const void *data, size_t len);
int flash_flatten(const struct device *dev, off_t offset, size_t size);

const struct flash_parameters *flash_get_parameters(const struct device *dev);
size_t flash_get_write_block_size(const struct device *dev);
int flash_get_page_info_by_offs(const struct device *dev, off_t offset,
                                struct flash_pages_info *info);

#ifdef __cplusplus
}
#endif

#endif /* AXDR_ZEPHYR_DRIVERS_FLASH_H_ */
