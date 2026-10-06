/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include <stdint.h>

/* Linker-defined load/run addresses for .ccmram_text. */
extern uint32_t _siccmram;
extern uint32_t _sccmram;
extern uint32_t _eccmram;

static void CCMRAM_Code_Init(void)
{
    uint32_t *Src;
    uint32_t *Dst;

    Src = &_siccmram;
    Dst = &_sccmram;

    while (Dst < &_eccmram)
    {
        *Dst++ = *Src++;
    }
}

/* startup_stm32g474xx.s calls __libc_init_array() after .data/.bss setup.
 * Register the CCMRAM copy in .preinit_array so RAM code is ready before main().
 */
__attribute__((used, section(".preinit_array")))
static void (*const CCMRAM_Code_Init_Entry)(void) = CCMRAM_Code_Init;
