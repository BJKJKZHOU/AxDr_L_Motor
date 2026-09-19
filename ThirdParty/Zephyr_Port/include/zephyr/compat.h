#ifndef AXDR_ZEPHYR_COMPAT_H_
#define AXDR_ZEPHYR_COMPAT_H_

#include <stddef.h>
#include <stdint.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define KB(x) ((x) * 1024UL)

#ifndef __packed
#define __packed __attribute__((packed))
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif

#define BUILD_ASSERT(condition, message) _Static_assert((condition), message)
#define ZTESTABLE_STATIC static

/* Evaluate to 1 only when the supplied configuration symbol expands to 1. */
#define ZEPHYR_PORT_ARG_PLACEHOLDER_1 0,
#define ZEPHYR_PORT_TAKE_SECOND_ARG(_ignored, value, ...) value
#define ZEPHYR_PORT_IS_ENABLED_EVAL(...) ZEPHYR_PORT_TAKE_SECOND_ARG(__VA_ARGS__ 1, 0)
#define ZEPHYR_PORT_IS_ENABLED_TEST(value) ZEPHYR_PORT_IS_ENABLED_EVAL(ZEPHYR_PORT_ARG_PLACEHOLDER_##value)
#define IS_ENABLED(config_macro) ZEPHYR_PORT_IS_ENABLED_TEST(config_macro)

#endif /* AXDR_ZEPHYR_COMPAT_H_ */
