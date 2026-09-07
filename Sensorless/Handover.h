/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef HANDOVER_H
#define HANDOVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float We_Err_F;
    float We_Err2_F;
    float PLL_Err2_F;
    float Theta_Err_F;
    float Theta_Err2_F;

    bool Speed_Valid;
    bool Theta_Valid;
    uint32_t Blend_Cnt;

} Handover_T;

void Handover_Reset(Handover_T *Handover);
void Handover_Compare_Reset(Handover_T *Handover);

void Handover_Speed_Compare(Handover_T *Handover,
                            float We_Ref,
                            float We_Obs,
                            float PLL_Err,
                            float Alpha);

void Handover_IF_Compare(Handover_T *Handover,
                         float Theta_IF,
                         float We_IF,
                         float Theta_Obs,
                         float We_Obs,
                         float PLL_Err,
                         float Alpha);

bool Handover_Speed_Stable(const Handover_T *Handover,
                           float We_Ref,
                           float We_Min,
                           float We_Mean_Ratio,
                           float We_Rms_Ratio,
                           float PLL_Rms_Max);

bool Handover_IF_Stable(const Handover_T *Handover,
                        float We_Ref,
                        float We_Min,
                        float We_Mean_Ratio,
                        float We_Rms_Ratio,
                        float PLL_Rms_Max,
                        float Theta_Rms_Max);

void Handover_Qualification_Accumulate(uint32_t *Count, uint32_t Limit, bool Good);

void Handover_Blend_Reset(Handover_T *Handover);
bool Handover_Blend_Run(Handover_T *Handover,
                        uint32_t Blend_Limit,
                        float Theta_IF,
                        float Theta_Obs,
                        float Id_IF,
                        float Iq_IF,
                        float *Theta_Use,
                        float *Id_Ref,
                        float *Iq_Ref);

float Handover_Ramp_Zero(float Value, float Step);
float Handover_Angle_Diff(float A, float B);

#endif /* HANDOVER_H */
