/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motion_Type.h"

#include "Motor_Type.h"
#include "control_params.h"
#include "motor_params.h"

Motion_Config_T Motion_Config =
{
    .Wm_Max = USER_WM_MAX_DEFAULT,
    .Wm_Acc = MOTION_ACC_RAD_S2,
    .Wm_Dec = MOTION_DEC_RAD_S2,
};
