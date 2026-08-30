/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Motion.h"

#include "Motor_Type.h"
#include "control_params.h"
#include "motor_params.h"

Motion_Config_T Motion_Config =
{
    .Wm_Max = USER_WM_MAX_DEFAULT,
    .Wm_Acc = MOTION_ACC_RAD_S2,
    .Wm_Dec = MOTION_DEC_RAD_S2,
};

bool Motion_Config_Set(float Wm_Max, float Wm_Acc, float Wm_Dec)
{
    Motion_Config_T Config;

    if (!(Wm_Max >= 0.0f) || !(Wm_Acc > 0.0f) || !(Wm_Dec > 0.0f))
    {
        return false;
    }

    if ((Wm_Max > User_Lim.Wm_Max) || (Wm_Max > Motor_Lim.Wm_Max))
    {
        return false;
    }

    Config.Wm_Max = Wm_Max;
    Config.Wm_Acc = Wm_Acc;
    Config.Wm_Dec = Wm_Dec;

    Motion_Config = Config;
    return true;
}
