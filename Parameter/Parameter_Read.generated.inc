/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */
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

case PARAM_SENSORLESS_STATE:
    Value->U8 = (uint8_t)Sensorless_State_Get();
    return PARAM_OK;
