/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "Enc_Control.h"

#include "Enc_Cal.h"
#include "Motor_Control.h"
#include "Motor_Cal.h"
#include "Servo_Phase.h"
#include "motor_work.h"

volatile Enc_Step_e Enc_Step = ENC_IDLE;

bool Enc_Control_Start(Enc_Mode_e Mode)
{
    if (Enc_Control_Active() || (Motor_Work_State != MOTOR_WORK_IDLE))
    {
        return false;
    }

    if (Mode == ENC_CAL)
    {
        if (!Enc_Cal_Start())
        {
            return false;
        }
        Enc_Step = ENC_CAL_RUN;
    }
    else if (Mode == ENC_PHASE)
    {
        /* A standalone phase search must not inherit an earlier CAL_DONE. */
        Enc_Cal.Step = CAL_IDLE;
        if (!Servo_Phase_Start())
        {
            return false;
        }
        Enc_Step = ENC_PHASE_RUN;
    }
    else
    {
        return false;
    }
    return true;
}

void Enc_Control_Abort(void)
{
    if (!Enc_Control_Active())
    {
        return;
    }

    if (Enc_Step == ENC_CAL_RUN)
    {
        Enc_Cal_Abort();
    }
    else if (Enc_Step == ENC_PHASE_RUN)
    {
        Servo_Phase_Abort();
    }
    /* A worker may still be fitting or writing Flash after the motor stops. */
    Motor_Cal.Valid = 0U;
    Enc_Step = ENC_STOP_WAIT;
}

void Enc_Control_Run(void)
{
    switch (Enc_Step)
    {
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
                Enc_Step = Servo_Phase_Start() ? ENC_PHASE_RUN : ENC_FAILED;
            }
            break;

        case ENC_PHASE_RUN:
            if (!Servo_Phase_Active())
            {
                Motor_Disable_Apply();
                if (Servo_Phase_Apply())
                {
                    if (Enc_Cal.Step == CAL_DONE)
                    {
                        Motor_Cal.Valid = 2U;
                    }
                    Enc_Step = Motor_Work_Request(MOTOR_WORK_SAVE_PHASE) ?
                               ENC_PHASE_SAVE : ENC_FAILED;
                }
                else
                {
                    Enc_Step = ENC_FAILED;
                }
            }
            break;

        case ENC_PHASE_SAVE:
            if (Motor_Work_State >= MOTOR_WORK_DONE)
            {
                Enc_Step = (Motor_Work_State == MOTOR_WORK_DONE) ? ENC_DONE : ENC_FAILED;
                Motor_Work_Ack();
            }
            break;

        case ENC_STOP_WAIT:
            Motor_Disable_Apply();
            if (Motor_Work_State != MOTOR_WORK_BUSY)
            {
                Motor_Work_Ack();
                Enc_Step = ENC_FAILED;
            }
            return;

        default:
            return;
    }

    if ((Enc_Step == ENC_PHASE_RUN) && (Motor_State_Get() == DISABLED))
    {
        if ((Motor_Enable() != MOTOR_OK) || (Motor_Start() != MOTOR_OK))
        {
            Enc_Control_Abort();
        }
    }
    if (!Enc_Control_Active() || (Enc_Step == ENC_STOP_WAIT))
    {
        if ((Enc_Step == ENC_FAILED) || (Enc_Step == ENC_STOP_WAIT))
        {
            Motor_Cal.Valid = 0U;
        }
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
        return Servo_Phase_Fast_Run(Theta_e, Id_Ref, Iq_Ref, Ualpha, Ubeta);
    }
    if (Enc_Step == ENC_CAL_RUN)
    {
        return Enc_Cal_Fast_Run(Theta_e, Ualpha, Ubeta);
    }
    return FAST_OFF;
}
