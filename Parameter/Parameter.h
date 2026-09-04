/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#ifndef PARAMETER_H
#define PARAMETER_H

#include <stdint.h>

#include "Motor_Type.h"
#include "Parameter_Id.generated.h"

#define PARAM_MOTOR_PP_MAX 64U

typedef enum
{
    PARAM_U8 = 0,
    PARAM_I8,
    PARAM_FLOAT,
    PARAM_I32,
    PARAM_U32,
    PARAM_POSITION,

} Parameter_Type_e;

typedef union
{
    uint8_t U8;
    int8_t I8;
    float F32;
    int32_t I32;
    uint32_t U32;
    Motor_Position_T Position;

} Parameter_Value_T;

typedef enum
{
    PARAM_OK = 0,
    PARAM_ERR_ID,
    PARAM_ERR_TYPE,
    PARAM_ERR_READ_ONLY,
    PARAM_ERR_VALUE,
    PARAM_ERR_STATE,

} Parameter_Status_e;

Parameter_Status_e Parameter_Read(uint16_t Id,
                                  Parameter_Type_e *Type,
                                  Parameter_Value_T *Value);
Parameter_Status_e Parameter_Write(uint16_t Id,
                                   Parameter_Type_e Type,
                                   Parameter_Value_T Value);
Parameter_Status_e Parameter_Write_Internal(uint16_t Id,
                                            Parameter_Type_e Type,
                                            Parameter_Value_T Value);
uint8_t Parameter_Value_Size(Parameter_Type_e Type);

#endif /* PARAMETER_H */
