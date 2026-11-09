/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Enc_Control.h"

#include "Enc_Cal.h"
#include "Enc_Home.h"
#include "Encoder.h"
#include "Motor_Control.h"
#include "Servo_Phase.h"
#include "motor_work.h"
#include "main.h"

volatile Enc_Step_e Enc_Step = ENC_IDLE;

bool Enc_Control_Start(Enc_Mode_e Mode)
{
    if (Mode == ENC_CAL)
    {
        if (!Enc_Cal_Start())
        {
            return false;
        }
        Enc_Step = ENC_CAL_RUN;
        Enc_Home.Valid = 0U;
    }
    else if (Mode == ENC_PHASE)
    {
        /* 独立寻相不继承完整校准流程的 CAL_DONE 状态。 */
        Enc_Cal.Step = CAL_IDLE;
        if (!Servo_Phase_Start(Motor_Cal.Valid == 2U))
        {
            return false;
        }
        Enc_Step = ENC_PHASE_RUN;
        Enc_Home.Valid = 0U;
    }
    else
    {
        if (!Enc_Home_Start())
        {
            return false;
        }
        Enc_Step = ENC_HOME_SEEK;
    }
    return true;
}

void Enc_Control_Abort(void)
{
    if (Enc_Step == ENC_CAL_RUN)
    {
        /* 取消后使后台拟合结果失效。 */
        Enc_Cal.Step = CAL_FAILED;
    }
    else if (Enc_Step == ENC_PHASE_RUN)
    {
        Servo_Phase.Step = PHASE_IDLE;
    }
    else if (Enc_Home_Active())
    {
        Motor_Cmd.Position_Target = Enc_Home.Position_Save;
        Motor_Cmd.Wm_Target = Enc_Home.Wm_Save;
    }
    /* 停机后等待后台拟合或 Flash 写入结束。 */
    Enc_Step = ENC_STOP_WAIT;
}

void Enc_Control_Run(void)
{
    uint32_t Primask;
    bool Dir_Changed;

    switch (Enc_Step)
    {
        case ENC_HOME_SEEK:
        case ENC_HOME_RETURN:
            Enc_Home_Run();
            if (!Enc_Home_Active())
            {
                Motor_Cmd.Wm_Target = Enc_Home.Wm_Save;
                if (Enc_Step == ENC_DONE)
                {
                    /* 回零完成后由位置环保持零点，禁能由外部命令决定。 */
                    return;
                }
                Motor_Cmd.Position_Target = Enc_Home.Position_Save;
            }
            break;

        case ENC_CAL_RUN:
            if (Enc_Cal.Step == CAL_FIT)
            {
                Motor_Disable_Apply();
            }
            Enc_Cal_Run();
            if (Enc_Cal.Step == CAL_FAILED)
            {
                Enc_Step = ENC_STOP_WAIT;
            }
            else if (Enc_Cal.Step == CAL_DONE)
            {
                Enc_Step = Servo_Phase_Start(true) ? ENC_PHASE_RUN : ENC_FAILED;
                if ((Enc_Step == ENC_PHASE_RUN) &&
                    ((Motor_Enable() != MOTOR_OK) || (Motor_Start() != MOTOR_OK)))
                {
                    Enc_Control_Abort();
                }
            }
            break;

        case ENC_PHASE_RUN:
            if ((Servo_Phase.Step < PHASE_ALIGN) || (Servo_Phase.Step > PHASE_ALIGN_FINAL))
            {
                Motor_Disable_Apply();
                if (Servo_Phase.Step == PHASE_DONE)
                {
                    /* 禁能时仍在采样，方向和零位一起更新。 */
                    Primask = __get_PRIMASK();
                    __disable_irq();
                    Dir_Changed = (Servo_Phase.Enc_Dir != Motor_Cal.Enc_Dir);
                    Motor_Cal.Enc_Dir = Servo_Phase.Enc_Dir;
                    Motor_Cal.Theta_Off = Servo_Phase.Theta_Off;
                    if (Motor_Cal.Valid != 2U)
                    {
                        Motor_Cal.Valid = 1U;
                    }
                    if (Dir_Changed)
                    {
                        Encoder_Position_Reset();
                    }
                    __set_PRIMASK(Primask);
                    Servo_Phase.Step = PHASE_IDLE;
                    if (Enc_Cal.Step == CAL_DONE)
                    {
                        Motor_Cal.Valid = 2U;
                    }
                    Motor_Work_Request(MOTOR_WORK_SAVE_PHASE);
                    Enc_Step = ENC_PHASE_SAVE;
                }
                else
                {
                    Servo_Phase.Step = PHASE_IDLE;
                    Enc_Step = ENC_FAILED;
                }
                return;
            }
            break;

        case ENC_PHASE_SAVE:
            if (Motor_Work_State >= MOTOR_WORK_DONE)
            {
                Enc_Step = (Motor_Work_State == MOTOR_WORK_DONE) ? ENC_DONE : ENC_FAILED;
                Motor_Work_State = MOTOR_WORK_IDLE;
            }
            return;

        case ENC_STOP_WAIT:
            if (Motor_Work_State != MOTOR_WORK_BUSY)
            {
                Motor_Work_State = MOTOR_WORK_IDLE;
                Enc_Step = ENC_FAILED;
            }
            return;

        default:
            return;
    }

    if (!Enc_Control_Active() || (Enc_Step == ENC_STOP_WAIT))
    {
        Motor_Disable_Apply();
    }
}

Motor_Fast_Mode_e Enc_Control_Fast_Run(float *Theta_e,
                                        float *Id_Ref,
                                        float *Iq_Ref,
                                        float *Ualpha,
                                        float *Ubeta)
{
    if (Enc_Step == ENC_PHASE_RUN)
    {
        return Servo_Phase_Fast_Run(Theta_e, Id_Ref, Iq_Ref);
    }
    if (Enc_Step == ENC_CAL_RUN)
    {
        return Enc_Cal_Fast_Run(Ualpha, Ubeta);
    }
    return FAST_OFF;
}
