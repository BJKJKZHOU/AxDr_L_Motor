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

    /* Mechanical_ESO.State.Theta: Mechanical ESO估算的单圈机械角度，范围[0,2π)，仅用于诊断与波形对比。 */
    PARAM_MECH_ESO_THETA = 0x0022U,

    /* Mechanical_ESO.State.Wm: Mechanical ESO估算的机械角速度。 */
    PARAM_MECH_ESO_WM = 0x0023U,

    /* Mechanical_ESO.State.Td: Mechanical ESO估算的负载/扰动转矩，仅用于诊断。 */
    PARAM_MECH_ESO_TD = 0x0024U,

    /* Mechanical_ESO.State.Error: Mechanical ESO的单圈角度观测误差，Theta_meas-Theta_hat回绕到[-π,π)。 */
    PARAM_MECH_ESO_ERROR = 0x0025U,

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

    /* Motor_Config.Align_Current_A: 该电机进行标准Align预定位时使用的d轴电流，供Flux/JB辨识与正常无感启动共用。 */
    PARAM_MOTOR_ALIGN_CURRENT = 0x0117U,

    /* Motor_Config.IF_Current_A: 该电机标准I/F开环拖动使用的q轴电流，供Flux/JB辨识与正常无感运行共用。 */
    PARAM_MOTOR_IF_CURRENT = 0x0118U,

    /* Motor_Config.RL_I_Peak_A: RL辨识注入电流目标峰值，为直流偏置与交流幅值之和，按65%直流、35%交流分配，默认1 A；独立于保护限值。最小约0.114286 A对应交流幅值0.04 A，启动时需低于有效电流限值。 */
    PARAM_MOTOR_RL_I_PEAK = 0x0119U,

    /* User_Lim.I_Max: 用户配置的最大相电流限制，实际限制不会超过硬件/固件Motor_Lim.I_Max。 */
    PARAM_LIMIT_I_MAX = 0x0201U,

    /* User_Lim.Wm_Max: 用户配置的最大机械角速度限制，实际限制不会超过Motor_Lim.Wm_Max。 */
    PARAM_LIMIT_WM_MAX = 0x0202U,

    /* Motor_I_Limit_Effective_Get(): 当前控制实际使用的相电流上限。 */
    PARAM_LIMIT_I_EFFECTIVE = 0x0203U,

    /* Motor_Wm_Limit_Effective_Get(): 当前控制使用的机械速度上限，为Motor与User速度限值的较小值，与目标转速无关。 */
    PARAM_LIMIT_WM_EFFECTIVE = 0x0204U,

    /* Protection_Vbus_Min_Get(): AxDr_L板级固定欠压保护阈值，只读。 */
    PARAM_LIMIT_VBUS_MIN = 0x0205U,

    /* Protection_Vbus_Max_Get(): AxDr_L板级固定过压保护阈值，只读。 */
    PARAM_LIMIT_VBUS_MAX = 0x0206U,

    /* Encoder_Config.Protocol: 运行前选择的编码器通信协议。 */
    PARAM_ENCODER_PROTOCOL = 0x0301U,

    /* Encoder.Ready: 编码器驱动是否已经完成初始化并可供控制使用。 */
    PARAM_ENCODER_READY = 0x0302U,

    /* Encoder.Valid: 当前编码器反馈数据是否有效。 */
    PARAM_ENCODER_VALID = 0x0303U,

    /* Encoder.Fault: 当前编码器故障标志。 */
    PARAM_ENCODER_FAULT = 0x0304U,

    /* Encoder_Config.SPI_Type: SPI协议下选择的编码器芯片协议。 */
    PARAM_ENCODER_SPI_TYPE = 0x0310U,

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

    /* Motion_Config.Wm_Acc: 运动规划器机械加速度限制，可在运行期间修改。 */
    PARAM_MOTION_WM_ACC = 0x0602U,

    /* Motion_Config.Wm_Dec: 运动规划器机械减速度限制，可在运行期间修改。 */
    PARAM_MOTION_WM_DEC = 0x0603U,

    /* Motion_Config.Te_Rate: 仅Torque模式的转矩指令变化率，运行及Stop降至零转矩共用；0关闭斜坡，默认0。可在运行期间修改，不影响Speed和Position模式。 */
    PARAM_MOTION_TE_RATE = 0x0604U,

    /* Motor_Mode: 当前选择的电机控制模式；与TORQUE/SPEED/POSITION/OPEN_LOOP/IDENT/SENSORLESS_SPEED/PHASE_SEARCH对应。 */
    PARAM_MOTOR_MODE = 0x0701U,

    /* Motor_Cmd.Te_Target: Torque模式的用户机械转矩目标。 */
    PARAM_TARGET_TORQUE = 0x0702U,

    /* Motor_Cmd.Wm_Target: 共用用户机械速度指令。Speed、Open-loop、Sensorless及Flux辨识使用有符号转速；Position使用绝对值规划运动，方向由目标位置决定，0使轨迹减速停止并保持位置。指令执行时受Motor与User速度限值约束，不覆盖原指令；不保存到Flash，上电为0。 */
    PARAM_TARGET_SPEED = 0x0703U,

    /* Motor_Cmd.Position_Target: Position模式的用户机械位置目标；wire格式固定为little-endian int32 Turn + float32 Theta，一次写入。 */
    PARAM_TARGET_POSITION = 0x0704U,

    /* Motor_State_Get(): 电机顶级生命周期状态，只读：DISABLED、ENABLED或RUN。状态迁移通过Action完成。 */
    PARAM_MOTOR_STATE = 0x0710U,

    /* Protection.Report: 当前提示/报告级事件位图。 */
    PARAM_EVENT_REPORT = 0x0801U,

    /* Protection.Warning: 当前警告级事件位图；警告不自动关闭电机。 */
    PARAM_EVENT_WARNING = 0x0802U,

    /* Protection.Fault: 当前错误级事件位图；非零时软件自动Disable并禁止重新Enable。 */
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

    /* Identification_JB_Valid_Get(): 最近一次J/B辨识结果是否有效。 */
    PARAM_IDENT_JB_VALID = 0x0920U,

    /* Identification_J_Get(): 最近一次J/B辨识得到的转动惯量J。 */
    PARAM_IDENT_J_RESULT = 0x0921U,

    /* Identification_B_Get(): 最近一次J/B辨识得到的粘性阻尼B。 */
    PARAM_IDENT_B_RESULT = 0x0922U,

    /* Ident_JB_Excite_Ratio: J/B辨识正弦机械速度激励幅值相对自动工作点速度的比例；默认0.20。 */
    PARAM_IDENT_JB_EXCITE_RATIO = 0x0923U,

    /* Ident_JB_Excite_Hz: J/B辨识正弦机械速度激励频率；默认3 Hz。 */
    PARAM_IDENT_JB_EXCITE_HZ = 0x0924U,

    /* Control_Current_Bw_Hz: 电流环设计带宽；写入后切换为Bandwidth并由当前Rs/Ld/Lq重新计算Id/Iq PI增益。 */
    PARAM_CTRL_CURRENT_BW_HZ = 0x0A01U,

    /* Id_Ctrl.Para.Kp: Id电流环实际Kp；直接写入后切换为Manual调参。 */
    PARAM_CTRL_ID_KP = 0x0A02U,

    /* Id_Ctrl.Para.Ki: Id电流环实际Ki；直接写入后切换为Manual调参。 */
    PARAM_CTRL_ID_KI = 0x0A03U,

    /* Iq_Ctrl.Para.Kp: Iq电流环实际Kp；直接写入后切换为Manual调参。 */
    PARAM_CTRL_IQ_KP = 0x0A04U,

    /* Iq_Ctrl.Para.Ki: Iq电流环实际Ki；直接写入后切换为Manual调参。 */
    PARAM_CTRL_IQ_KI = 0x0A05U,

    /* Control_Current_Tune_Source: 当前电流环增益来源；切到Bandwidth时按模型计算PI，保存配置命令不覆盖手调增益；切回Manual恢复Flash中已保存的Id/Iq增益，全组未保存时沿用当前值。 */
    PARAM_CTRL_CURRENT_SOURCE = 0x0A06U,

    /* Control_Speed_Bw_Hz: 速度环设计带宽；写入后切换为Bandwidth并由当前J/B/Flux/Pp重新计算速度PI增益。 */
    PARAM_CTRL_SPEED_BW_HZ = 0x0A10U,

    /* Speed_Ctrl.Para.Kp: 速度环实际Kp；直接写入后切换为Manual调参。 */
    PARAM_CTRL_SPEED_KP = 0x0A11U,

    /* Speed_Ctrl.Para.Ki: 速度环实际Ki；直接写入后切换为Manual调参。 */
    PARAM_CTRL_SPEED_KI = 0x0A12U,

    /* Control_Speed_Tune_Source: 当前速度环增益来源；切到Bandwidth时按模型计算PI，保存配置命令不覆盖手调增益；切回Manual恢复Flash中已保存的Kp/Ki，全组未保存时沿用当前值。 */
    PARAM_CTRL_SPEED_SOURCE = 0x0A13U,

    /* Pos_Ctrl.Para.Kp: 位置环P增益，单位为(rad/s)/rad。 */
    PARAM_CTRL_POSITION_KP = 0x0A20U,

    /* Mechanical_ESO_Bw_Hz: 20 kHz机械ESO设计带宽；写入后按当前J/B/Kt刷新Observer增益。 */
    PARAM_CTRL_MECH_ESO_BW_HZ = 0x0A30U,

    /* MOTOR_CMD_ENABLE: 使能电机功率级。 */
    ACTION_MOTOR_ENABLE = 0x1001U,

    /* MOTOR_CMD_RUN: 按当前Motor Mode启动运行；PHASE_SEARCH模式下作为异步寻相启动动作。 */
    ACTION_MOTOR_RUN = 0x1002U,

    /* MOTOR_CMD_STOP: 请求停止当前运行；运动模式按当前减速度受控停稳后回到ENABLED，Disable仍用于立即关闭驱动。 */
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

    /* MOTOR_CMD_IDENT_START: 启动J/B辨识。 */
    ACTION_IDENT_JB_START = 0x1105U,

    /* MOTOR_CMD_PROTECTION_CLEAR: 在DISABLED状态清除可清除的保护状态。 */
    ACTION_PROTECTION_CLEAR = 0x1201U,

    /* MOTOR_CMD_PARAMETER_SAVE: 将当前持久化参数显式保存到NVS；仅DISABLED状态允许执行。 */
    ACTION_PARAMETER_SAVE = 0x1202U,

} Parameter_Id_e;

#endif /* PARAMETER_GENERATED_H */
