/* Generated from Parameter/action_objects.yaml. DO NOT EDIT. */
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
