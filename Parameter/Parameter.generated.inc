/* Generated from Parameter/parameter.yaml. DO NOT EDIT. */

#if defined(PARAM_GENERATE_TABLE)
{ PARAM_ADC_IA, PARAM_FLOAT, &ADC.Ia_A, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_ADC_IB, PARAM_FLOAT, &ADC.Ib_A, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_ADC_IC, PARAM_FLOAT, &ADC.Ic_A, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_ADC_VBUS, PARAM_FLOAT, &ADC.Vbus_V, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_ID, PARAM_FLOAT, &Motor_Run.Id, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_IQ, PARAM_FLOAT, &Motor_Run.Iq, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_UD, PARAM_FLOAT, &Motor_Run.Ud, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_UQ, PARAM_FLOAT, &Motor_Run.Uq, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_THETA_E, PARAM_FLOAT, &Motor_Run.Theta_e, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_UALPHA, PARAM_FLOAT, &Motor_Run.Ualpha, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_UBETA, PARAM_FLOAT, &Motor_Run.Ubeta, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_OBS_THETA, PARAM_FLOAT, &Flux_PLL.State.Theta, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_OBS_WE, PARAM_FLOAT, &Flux_PLL.State.We, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_OBS_PLL_ERR, PARAM_FLOAT, &Flux_PLL.State.Err, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_OBS_FLUX_ERR, PARAM_FLOAT, &Flux_Obs.State.Flux_Err, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_MOTOR_PP, PARAM_U8, &Motor_Para.Pp, 1.0f, 64.0f, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_PP },
{ PARAM_MOTOR_RS, PARAM_FLOAT, &Motor_Para.Rs, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_RL },
{ PARAM_MOTOR_LD, PARAM_FLOAT, &Motor_Para.Ld, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_RL },
{ PARAM_MOTOR_LQ, PARAM_FLOAT, &Motor_Para.Lq, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_RL },
{ PARAM_MOTOR_FLUX, PARAM_FLOAT, &Motor_Para.Flux, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_FLUX },
{ PARAM_MOTOR_J, PARAM_FLOAT, &Motor_Para.J, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_JB },
{ PARAM_MOTOR_B, PARAM_FLOAT, &Motor_Para.B, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_MOTOR_JB },
{ PARAM_MOTOR_DIR, PARAM_I8, &Motor_Config.Dir, -1.0f, 1.0f, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_NONE },
{ PARAM_LIMIT_I_MAX, PARAM_FLOAT, &User_Lim.I_Max, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_NONE },
{ PARAM_LIMIT_WM_MAX, PARAM_FLOAT, &User_Lim.Wm_Max, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_NONE },
{ PARAM_LIMIT_I_EFFECTIVE, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_LIMIT_WM_EFFECTIVE, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_ENCODER_TYPE, PARAM_U8, &Encoder_Config.Type, (float)ENC_MT6816, (float)ENC_MT6835, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_ENCODER },
{ PARAM_ENCODER_READY, PARAM_U8, &Encoder.Ready, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_ENCODER_VALID, PARAM_U8, &Encoder.Valid, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_ENCODER_FAULT, PARAM_U8, &Encoder.Fault, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_PHASE_I_SEARCH, PARAM_FLOAT, &Servo_Phase_Config.I_Search_A, 0.0f, (float)SERVO_PHASE_I_MAX_A, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_NONE },
{ PARAM_CAL_VALID, PARAM_U8, &Motor_Cal.Valid, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_CAL_ENC_DIR, PARAM_I8, &Motor_Cal.Enc_Dir, -1.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_CAL_THETA_OFF, PARAM_FLOAT, &Motor_Cal.Theta_Off, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_PHASE_THETA_OFF_ERROR, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_PHASE_VERIFY_MOVE, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_POSITION, PARAM_POSITION, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_RUN_WM, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_REF_IQ, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_REF_WM, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_REF_POSITION, PARAM_POSITION, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_MOTION_WM_MAX, PARAM_FLOAT, &Motion_Config.Wm_Max, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_MOTION_WM_ACC, PARAM_FLOAT, &Motion_Config.Wm_Acc, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_MOTION_WM_DEC, PARAM_FLOAT, &Motion_Config.Wm_Dec, 0.0f, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_MOTOR_MODE, PARAM_U8, &Motor_Mode, (float)TORQUE, (float)PHASE_SEARCH, PARAM_FLAG_HOST_WRITE | PARAM_FLAG_DISABLED_ONLY, PARAM_CHANGE_NONE },
{ PARAM_TARGET_TORQUE, PARAM_FLOAT, &Motor_Cmd.Te_Target, -INFINITY, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_TARGET_SPEED, PARAM_FLOAT, &Motor_Cmd.Wm_Target, -INFINITY, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_TARGET_POSITION, PARAM_POSITION, &Motor_Cmd.Position_Target, -INFINITY, INFINITY, PARAM_FLAG_HOST_WRITE, PARAM_CHANGE_NONE },
{ PARAM_MOTOR_STATE, PARAM_U8, NULL, (float)DISABLED, (float)RUN, 0U, PARAM_CHANGE_NONE },
{ PARAM_EVENT_REPORT, PARAM_U32, &Protection.Report, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_EVENT_WARNING, PARAM_U32, &Protection.Warning, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_EVENT_ERROR, PARAM_U32, &Protection.Stop, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_EVENT_TRIP, PARAM_U32, &Protection.Trip, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_IDENT_RS_LS_VALID, PARAM_U8, NULL, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_IDENT_RS_RESULT, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_IDENT_LS_RESULT, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },
{ PARAM_IDENT_FLUX_VALID, PARAM_U8, NULL, 0.0f, 1.0f, 0U, PARAM_CHANGE_NONE },
{ PARAM_IDENT_FLUX_RESULT, PARAM_FLOAT, NULL, -INFINITY, INFINITY, 0U, PARAM_CHANGE_NONE },

#elif defined(PARAM_GENERATE_VALIDATE)
case PARAM_MOTOR_RS:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_LD:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_LQ:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_FLUX:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_J:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_DIR:
    if ((Value.I8 != (int8_t)-1.0f) && (Value.I8 != (int8_t)1.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_LIMIT_I_MAX:
    if ((Number <= 0.0f) ||
        (Number > (float)Motor_Lim.I_Max))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_LIMIT_WM_MAX:
    if ((Number <= 0.0f) ||
        (Number > (float)Motor_Lim.Wm_Max))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_ENCODER_TYPE:
    if ((Value.U8 != (uint8_t)ENC_MT6816) && (Value.U8 != (uint8_t)ENC_MT6835))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_PHASE_I_SEARCH:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTION_WM_MAX:
    if ((Number > (float)User_Lim.Wm_Max) ||
        (Number > (float)Motor_Lim.Wm_Max))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTION_WM_ACC:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTION_WM_DEC:
    if ((Number <= 0.0f))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_MODE:
    if ((Value.U8 != (uint8_t)TORQUE) && (Value.U8 != (uint8_t)SPEED) && (Value.U8 != (uint8_t)POSITION) && (Value.U8 != (uint8_t)OPEN_LOOP) && (Value.U8 != (uint8_t)IDENT) && (Value.U8 != (uint8_t)SENSORLESS_SPEED) && (Value.U8 != (uint8_t)PHASE_SEARCH))
    {
        return PARAM_ERR_VALUE;
    }
    break;

case PARAM_MOTOR_STATE:
    if ((Value.U8 != (uint8_t)DISABLED) && (Value.U8 != (uint8_t)ENABLED) && (Value.U8 != (uint8_t)RUN))
    {
        return PARAM_ERR_VALUE;
    }
    break;

#elif defined(PARAM_GENERATE_READ)
case PARAM_LIMIT_I_EFFECTIVE:
    Value->F32 = (float)Motor_I_Limit_Effective_Get();
    return PARAM_OK;

case PARAM_LIMIT_WM_EFFECTIVE:
    Value->F32 = (float)Motor_Wm_Limit_Effective_Get();
    return PARAM_OK;

case PARAM_PHASE_THETA_OFF_ERROR:
    Value->F32 = (float)Servo_Phase_Theta_Off_Error_Get();
    return PARAM_OK;

case PARAM_PHASE_VERIFY_MOVE:
    Value->F32 = (float)Servo_Phase_Verify_Move_Get();
    return PARAM_OK;

case PARAM_RUN_POSITION:
    Value->Position = Parameter_Run_Position_Get();
    return PARAM_OK;

case PARAM_RUN_WM:
    Value->F32 = (float)Motor_Wm_Get();
    return PARAM_OK;

case PARAM_REF_IQ:
    Value->F32 = (float)Motor_Iq_Ref_Get();
    return PARAM_OK;

case PARAM_REF_WM:
    Value->F32 = (float)Motor_Wm_Ref_Get();
    return PARAM_OK;

case PARAM_REF_POSITION:
    Value->Position = Motor_Position_Ref_Get();
    return PARAM_OK;

case PARAM_MOTOR_STATE:
    Value->U8 = (uint8_t)Motor_State_Get();
    return PARAM_OK;

case PARAM_IDENT_RS_LS_VALID:
    Value->U8 = (uint8_t)Identification_Rs_Ls_Valid_Get();
    return PARAM_OK;

case PARAM_IDENT_RS_RESULT:
    Value->F32 = (float)Identification_Rs_Get();
    return PARAM_OK;

case PARAM_IDENT_LS_RESULT:
    Value->F32 = (float)Identification_Ls_Get();
    return PARAM_OK;

case PARAM_IDENT_FLUX_VALID:
    Value->U8 = (uint8_t)Identification_Flux_Valid_Get();
    return PARAM_OK;

case PARAM_IDENT_FLUX_RESULT:
    Value->F32 = (float)Identification_Flux_Get();
    return PARAM_OK;

#elif defined(PARAM_GENERATE_ACTION)
case ACTION_MOTOR_ENABLE:
    Msg.Cmd = (ULONG)MOTOR_CMD_ENABLE;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_MOTOR_RUN:
    Msg.Cmd = (ULONG)MOTOR_CMD_RUN;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_MOTOR_STOP:
    Msg.Cmd = (ULONG)MOTOR_CMD_STOP;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_MOTOR_DISABLE:
    Msg.Cmd = (ULONG)MOTOR_CMD_DISABLE;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_IDENT_RS_LS_START:
    Msg.Cmd = (ULONG)MOTOR_CMD_IDENT_START;
    Msg.Arg = (ULONG)IDENT_RS_LS;
    break;

case ACTION_IDENT_FLUX_START:
    Msg.Cmd = (ULONG)MOTOR_CMD_IDENT_START;
    Msg.Arg = (ULONG)IDENT_FLUX;
    break;

case ACTION_IDENT_ABORT:
    Msg.Cmd = (ULONG)MOTOR_CMD_IDENT_ABORT;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_IDENT_APPLY:
    Msg.Cmd = (ULONG)MOTOR_CMD_IDENT_APPLY;
    Msg.Arg = (ULONG)0U;
    break;

case ACTION_PROTECTION_CLEAR:
    Msg.Cmd = (ULONG)MOTOR_CMD_PROTECTION_CLEAR;
    Msg.Arg = (ULONG)0U;
    break;

#elif defined(PARAM_GENERATE_PLOT)
case PARAM_ADC_IA:
    *Scale = 0.001f;
    return &ADC.Ia_A;

case PARAM_ADC_IB:
    *Scale = 0.001f;
    return &ADC.Ib_A;

case PARAM_ADC_IC:
    *Scale = 0.001f;
    return &ADC.Ic_A;

case PARAM_ADC_VBUS:
    *Scale = 0.0f;
    return &ADC.Vbus_V;

case PARAM_RUN_ID:
    *Scale = 0.001f;
    return &Motor_Run.Id;

case PARAM_RUN_IQ:
    *Scale = 0.001f;
    return &Motor_Run.Iq;

case PARAM_RUN_UD:
    *Scale = 0.001f;
    return &Motor_Run.Ud;

case PARAM_RUN_UQ:
    *Scale = 0.001f;
    return &Motor_Run.Uq;

case PARAM_RUN_THETA_E:
    *Scale = 0.0002f;
    return &Motor_Run.Theta_e;

case PARAM_RUN_UALPHA:
    *Scale = 0.001f;
    return &Motor_Run.Ualpha;

case PARAM_RUN_UBETA:
    *Scale = 0.001f;
    return &Motor_Run.Ubeta;

case PARAM_OBS_THETA:
    *Scale = 0.0002f;
    return &Flux_PLL.State.Theta;

case PARAM_OBS_WE:
    *Scale = 0.1f;
    return &Flux_PLL.State.We;

case PARAM_OBS_PLL_ERR:
    *Scale = 0.0001f;
    return &Flux_PLL.State.Err;

case PARAM_OBS_FLUX_ERR:
    *Scale = 1e-09f;
    return &Flux_Obs.State.Flux_Err;

#else
#error "Parameter.generated.inc section not selected"
#endif
