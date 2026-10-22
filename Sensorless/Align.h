/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef ALIGN_H
#define ALIGN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    struct
    {
        float Id_A;
        uint32_t Hold_Cnt;
    } Para;

    struct
    {
        float Id_Ref;
        float Iq_Ref;
    } Sig;

    struct
    {
        uint32_t Cnt;
    } State;

} Align_T;

bool Align_Run(Align_T *Align);

#endif /* ALIGN_H */
