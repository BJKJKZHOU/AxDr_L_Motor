/* Generated from Parameter/parameter.yaml. DO NOT EDIT. */
#ifndef PARAMETER_GENERATED_H
#define PARAMETER_GENERATED_H

typedef enum
{
    /* ADC.Ia_A: ADC采样得到的A相电流反馈。 */
    PARAM_ADC_IA = 0x0001U,

    /* ADC.Ib_A: ADC采样得到的B相电流反馈。 */
    PARAM_ADC_IB = 0x0002U,

    /* ADC.Ic_A: 由相电流采样链得到的C相电流反馈。 */
    PARAM_ADC_IC = 0x0003U,

    /* ADC.Vbus_V: 直流母线电压采样值。 */
    PARAM_ADC_VBUS = 0x0004U,

    /* Motor_Run.Id: FOC Park变换后的d轴实际电流反馈。 */
    PARAM_RUN_ID = 0x0010U,

    /* Motor_Run.Iq: FOC Park变换后的q轴实际电流反馈。 */
    PARAM_RUN_IQ = 0x0011U,

    /* Motor_Run.Ud: FOC电流环输出的d轴电压。 */
    PARAM_RUN_UD = 0x0012U,

    /* Motor_Run.Uq: FOC电流环输出的q轴电压。 */
    PARAM_RUN_UQ = 0x0013U,

    /* Motor_Run.Theta_e: 当前FOC实际使用的电角度。 */
    PARAM_RUN_THETA_E = 0x0014U,

    /* Motor_Run.Ualpha: 上一快环实际送入SVPWM的alpha轴电压命令。 */
    PARAM_RUN_UALPHA = 0x0015U,

    /* Motor_Run.Ubeta: 上一快环实际送入SVPWM的beta轴电压命令。 */
    PARAM_RUN_UBETA = 0x0016U,

    /* Flux_PLL.State.Theta: 磁链观测器PLL估算的电角度。 */
    PARAM_OBS_THETA = 0x0020U,

    /* Flux_PLL.State.We: 磁链观测器PLL估算的电角速度。 */
    PARAM_OBS_WE = 0x0021U,

    /* Motor_Para.Pp: 电机极对数。 */
    PARAM_MOTOR_PP = 0x0101U,

    /* Motor_Para.Rs: 电机相电阻Rs，供电机模型、电流控制及无感相关算法使用。 */
    PARAM_MOTOR_RS = 0x0110U,

    /* Motor_Para.Ld: 电机d轴电感Ld。 */
    PARAM_MOTOR_LD = 0x0111U,

    /* Motor_Para.Lq: 电机q轴电感Lq。 */
    PARAM_MOTOR_LQ = 0x0112U,

    /* Motor_Para.Flux: 电机永磁磁链参数。 */
    PARAM_MOTOR_FLUX = 0x0113U,

    /* Motor_Para.J: 电机及等效负载转动惯量参数J。 */
    PARAM_MOTOR_J = 0x0114U,

    /* Motor_Para.B: 电机粘性阻尼/摩擦模型参数B。 */
    PARAM_MOTOR_B = 0x0115U,

    /* Motor_Config.Dir: 用户机械正方向与内部控制方向之间的符号映射，只允许-1或+1。 */
    PARAM_MOTOR_DIR = 0x0116U,

    /* User_Lim.I_Max: 用户配置的最大相电流限制，实际限制不会超过硬件/固件Motor_Lim.I_Max。 */
    PARAM_LIMIT_I_MAX = 0x0201U,

    /* User_Lim.Wm_Max: 用户配置的最大机械角速度限制，实际限制不会超过Motor_Lim.Wm_Max。 */
    PARAM_LIMIT_WM_MAX = 0x0202U,

    /* Motor_I_Limit_Effective_Get(): 当前控制实际使用的相电流上限。 */
    PARAM_LIMIT_I_EFFECTIVE = 0x0203U,

    /* Motor_Wm_Limit_Effective_Get(): 当前控制实际使用的机械速度上限，包含Motor、User与Motion限制。 */
    PARAM_LIMIT_WM_EFFECTIVE = 0x0204U,

    /* Encoder_Config.Type: 运行前选择的编码器类型。 */
    PARAM_ENCODER_TYPE = 0x0301U,

    /* Encoder.Ready: 编码器驱动是否已经完成初始化并可供控制使用。 */
    PARAM_ENCODER_READY = 0x0302U,

    /* Encoder.Valid: 当前编码器反馈数据是否有效。 */
    PARAM_ENCODER_VALID = 0x0303U,

    /* Encoder.Fault: 当前编码器故障标志。 */
    PARAM_ENCODER_FAULT = 0x0304U,

    /* Servo_Phase_Config.I_Search_A: 伺服寻相过程使用的持续寻相电流。 */
    PARAM_PHASE_I_SEARCH = 0x0401U,

    /* Motor_Cal.Valid: 当前伺服相位/编码器校准结果是否有效。 */
    PARAM_CAL_VALID = 0x0402U,

    /* Motor_Cal.Enc_Dir: 寻相得到的编码器方向符号，按工程约定用于机械方向对齐。 */
    PARAM_CAL_ENC_DIR = 0x0403U,

    /* Motor_Cal.Theta_Off: 寻相/校准得到的编码器到电角度的零位偏置。 */
    PARAM_CAL_THETA_OFF = 0x0404U,

    /* Servo_Phase_Theta_Off_Error_Get(): 最近一次伺服寻相正反扫描得到的电角零偏一致性误差。 */
    PARAM_PHASE_THETA_OFF_ERROR = 0x0410U,

    /* Servo_Phase_Verify_Move_Get(): 最近一次伺服寻相最终正Iq方向验证得到的机械位移。 */
    PARAM_PHASE_VERIFY_MOVE = 0x0411U,

    /* Parameter_Run_Position_Get(): 当前用户机械坐标下的位置反馈。 */
    PARAM_RUN_POSITION = 0x0501U,

    /* Motor_Wm_Get(): 当前用户机械坐标下的机械角速度反馈。 */
    PARAM_RUN_WM = 0x0502U,

    /* Motor_Iq_Ref_Get(): 最近一次快环实际使用的q轴电流参考。 */
    PARAM_REF_IQ = 0x0510U,

    /* Motor_Wm_Ref_Get(): 当前速度控制实际使用的用户机械速度参考。 */
    PARAM_REF_WM = 0x0511U,

    /* Motor_Position_Ref_Get(): 当前位置轨迹实际使用的用户机械位置参考。 */
    PARAM_REF_POSITION = 0x0512U,

    /* Motion_Config.Wm_Max: 运动规划器允许使用的最大机械角速度。 */
    PARAM_MOTION_WM_MAX = 0x0601U,

    /* Motion_Config.Wm_Acc: 运动规划器机械加速度限制，可在运行期间修改。 */
    PARAM_MOTION_WM_ACC = 0x0602U,

    /* Motion_Config.Wm_Dec: 运动规划器机械减速度限制，可在运行期间修改。 */
    PARAM_MOTION_WM_DEC = 0x0603U,

    /* Motor_Mode: 当前选择的电机控制模式；与TORQUE/SPEED/POSITION/OPEN_LOOP/IDENT/SENSORLESS_SPEED/PHASE_SEARCH对应。 */
    PARAM_MOTOR_MODE = 0x0701U,

    /* Motor_Cmd.Te_Target: Torque模式的用户机械转矩目标。 */
    PARAM_TARGET_TORQUE = 0x0702U,

    /* Motor_Cmd.Wm_Target: Speed、Open-loop、Sensorless及Flux辨识流程使用的用户机械速度目标。 */
    PARAM_TARGET_SPEED = 0x0703U,

    /* Motor_Cmd.Position_Target: Position模式的用户机械位置目标；wire格式固定为little-endian int32 Turn + float32 Theta，一次写入。 */
    PARAM_TARGET_POSITION = 0x0704U,

    /* Motor_State_Get(): 电机顶级生命周期状态，只读：DISABLED、ENABLED或RUN。状态迁移通过Action完成。 */
    PARAM_MOTOR_STATE = 0x0710U,

    /* Protection.Report: 当前提示/报告级事件位图。 */
    PARAM_EVENT_REPORT = 0x0801U,

    /* Protection.Warning: 当前警告级事件位图；警告不自动关闭电机。 */
    PARAM_EVENT_WARNING = 0x0802U,

    /* Protection.Stop: 当前错误级事件位图；非零时软件自动Disable并禁止重新Enable。 */
    PARAM_EVENT_ERROR = 0x0803U,

    /* Protection.Trip: 当前硬件快速关断级事件位图。 */
    PARAM_EVENT_TRIP = 0x0804U,

    /* Identification_Rs_Ls_Valid_Get(): 最近一次Rs/Ls辨识结果是否有效。 */
    PARAM_IDENT_RS_LS_VALID = 0x0901U,

    /* Identification_Rs_Get(): 最近一次Rs/Ls辨识得到的相电阻结果。 */
    PARAM_IDENT_RS_RESULT = 0x0902U,

    /* Identification_Ls_Get(): 最近一次Rs/Ls辨识得到的等效相电感结果。 */
    PARAM_IDENT_LS_RESULT = 0x0903U,

    /* Identification_Flux_Valid_Get(): 最近一次Flux辨识结果是否有效。 */
    PARAM_IDENT_FLUX_VALID = 0x0910U,

    /* Identification_Flux_Get(): 最近一次Flux辨识得到的永磁磁链结果。 */
    PARAM_IDENT_FLUX_RESULT = 0x0911U,

    /* Identification_Fail_Reason_Get(): 最近一次辨识失败原因；0表示无失败，非零仅用于最终诊断。 */
    PARAM_IDENT_FAIL_REASON = 0x0912U,

    /* Ident_IF_Current_A: Flux辨识标准I/F开环启动使用的q轴电流，由Host按被测电机显式给定；与Imax安全上限独立。 */
    PARAM_IDENT_IF_CURRENT = 0x0913U,

    /* MOTOR_CMD_ENABLE: 使能电机功率级。 */
    ACTION_MOTOR_ENABLE = 0x1001U,

    /* MOTOR_CMD_RUN: 按当前Motor Mode启动运行；PHASE_SEARCH模式下作为异步寻相启动动作。 */
    ACTION_MOTOR_RUN = 0x1002U,

    /* MOTOR_CMD_STOP: 停止当前运行并回到ENABLED。 */
    ACTION_MOTOR_STOP = 0x1003U,

    /* MOTOR_CMD_DISABLE: 立即关闭PWM并进入DISABLED。 */
    ACTION_MOTOR_DISABLE = 0x1004U,

    /* MOTOR_CMD_IDENT_START: 启动Rs/Ls辨识。 */
    ACTION_IDENT_RS_LS_START = 0x1101U,

    /* MOTOR_CMD_IDENT_START: 启动Flux辨识。 */
    ACTION_IDENT_FLUX_START = 0x1102U,

    /* MOTOR_CMD_IDENT_ABORT: 中止当前辨识。 */
    ACTION_IDENT_ABORT = 0x1103U,

    /* MOTOR_CMD_IDENT_APPLY: 将最近一次有效辨识结果应用到Motor参数。 */
    ACTION_IDENT_APPLY = 0x1104U,

    /* MOTOR_CMD_PROTECTION_CLEAR: 在DISABLED状态清除可清除的保护状态。 */
    ACTION_PROTECTION_CLEAR = 0x1201U,

} Parameter_Id_e;

#endif /* PARAMETER_GENERATED_H */
