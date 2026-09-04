/* Generated from Parameter/parameter_objects.yaml. DO NOT EDIT. */
case PARAM_MOTOR_STATE:
    Value->U8 = (uint8_t)Motor_State_Get();
    return PARAM_OK;

case PARAM_SENSORLESS_STATE:
    Value->U8 = (uint8_t)Sensorless_State_Get();
    return PARAM_OK;
