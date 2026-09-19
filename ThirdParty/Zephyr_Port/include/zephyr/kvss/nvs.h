#ifndef AXDR_ZEPHYR_KVSS_NVS_H_
#define AXDR_ZEPHYR_KVSS_NVS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "tx_api.h"
#include <zephyr/compat.h>

#ifdef __cplusplus
extern "C" {
#endif

struct device;
struct flash_parameters;

struct k_mutex {
    TX_MUTEX mutex;
    uint8_t initialized;
};

#define K_FOREVER TX_WAIT_FOREVER

static inline int k_mutex_init(struct k_mutex *mutex)
{
    if (mutex->initialized != 0U)
    {
        return 0;
    }

    if (tx_mutex_create(&mutex->mutex, (CHAR *)"nvs", TX_NO_INHERIT) != TX_SUCCESS)
    {
        return -1;
    }

    mutex->initialized = 1U;
    return 0;
}

static inline int k_mutex_lock(struct k_mutex *mutex, ULONG timeout)
{
    return (tx_mutex_get(&mutex->mutex, timeout) == TX_SUCCESS) ? 0 : -1;
}

static inline int k_mutex_unlock(struct k_mutex *mutex)
{
    return (tx_mutex_put(&mutex->mutex) == TX_SUCCESS) ? 0 : -1;
}

struct nvs_fs {
    off_t offset;
    uint32_t ate_wra;
    uint32_t data_wra;
    uint32_t sector_size;
    uint16_t sector_count;
    bool ready;
    struct k_mutex nvs_lock;
    const struct device *flash_device;
    const struct flash_parameters *flash_parameters;
#if CONFIG_NVS_LOOKUP_CACHE
    uint32_t lookup_cache[CONFIG_NVS_LOOKUP_CACHE_SIZE];
#endif
};

int nvs_mount(struct nvs_fs *fs);
int nvs_clear(struct nvs_fs *fs);
ssize_t nvs_write(struct nvs_fs *fs, uint16_t id, const void *data, size_t len);
int nvs_delete(struct nvs_fs *fs, uint16_t id);
ssize_t nvs_read(struct nvs_fs *fs, uint16_t id, void *data, size_t len);
ssize_t nvs_read_hist(struct nvs_fs *fs, uint16_t id, void *data, size_t len, uint16_t cnt);
ssize_t nvs_calc_free_space(struct nvs_fs *fs);
size_t nvs_sector_max_data_size(struct nvs_fs *fs);
int nvs_sector_use_next(struct nvs_fs *fs);

#ifdef __cplusplus
}
#endif

#endif /* AXDR_ZEPHYR_KVSS_NVS_H_ */
