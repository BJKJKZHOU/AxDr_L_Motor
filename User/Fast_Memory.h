/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef FAST_MEMORY_H
#define FAST_MEMORY_H

/* Execute selected hard-real-time code from STM32G4 CCMRAM. */
#define FAST_CODE __attribute__((section(".ccmram_text")))

#endif /* FAST_MEMORY_H */
