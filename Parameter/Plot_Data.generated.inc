/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */
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

case PARAM_RUN_THETA_M:
    *Scale = 0.0002f;
    return &Motor_Run.Theta_m;

case PARAM_RUN_WM:
    *Scale = 0.1f;
    return &Motor_Run.Wm;
