# Embedded Motor-Control C Code Style

> 适用于 STM32 / RTOS / FOC / 伺服控制工程。
>
> 目标不是追求“软件工程形式完整”，而是让代码尽量接近控制框图和 Simulink 信号流：变量能对应物理量，结构能对应控制模块，调用关系能对应实际执行周期。

## 1. 总原则

1. **控制含义优先于软件抽象。**
   - 优先让代码表达电流、速度、位置、角度、电压、状态和控制流程。
   - 不为“接口统一”“层次完整”“面向对象感”增加无实际意义的包装。

2. **简单事情保持简单。**
   - 一两行明确赋值，不因为“以后可能复用”就包装函数。
   - 能直接读懂的状态改变，优先保留在具有真实场景语义的调用处。

3. **一个数据只保存它真正需要存在的位置。**
   - 不重复保存可以直接推导的数据。
   - 不保存只在一次函数调用中使用的中间变量。
   - 不保存硬件寄存器已经能直接表达的状态副本。

4. **实时热路径优先考虑执行效率和可读性。**
   - ADC / PWM 高频 ISR 可以直接读寄存器、直接展开公式、使用局部变量、直接写 PWM。
   - 不为了抽象层次强制经过多层函数和对象包装。

5. **修改已有工程时优先保持现有行为。**
   - 除非任务明确要求，不顺手重构无关代码。
   - 不主动引入新的框架、状态机、生命周期接口或泛化层。

---

## 2. 命名风格

### 2.1 基本形式

采用类似 Simulink / 控制工程的命名方式：

```c
Current_Loop
Speed_Loop
Position_Loop
Motor_Ctrl
Motor_State
Phase_Find
Enc_Cal
RL_Id
Loop_Sweep
```

规则：

- 单词首字母大写。
- 单词之间使用 `_`。
- 常见技术缩写保持原缩写：`PID`、`ADC`、`PWM`、`SPI`、`FOC`、`SVPWM`、`DMA`。
- 避免纯软件式超长描述名。

推荐：

```c
Motor_Enable()
Motor_Disable()
Motor_Stop()
Motor_Ctrl_Run()
PID_Run()
Enc_Cal_Run()
Phase_Find_Run()
```

避免：

```c
MotorControlManagerInitializeControllerState()
MotorCurrentFeedbackProcessingHandler()
ServoControlServiceResetInternalState()
```

### 2.2 优先使用控制理论和物理符号

推荐：

```c
Ia
Ib
Ic
Id
Iq
Ud
Uq
Theta_e
Theta_m
Wm
Rs
Ld
Lq
Psi_f
Jm
Ts
```

避免为了自解释而写成：

```c
MotorPhaseACurrentFeedback
ElectricalRotorPositionRadians
MechanicalAngularVelocityFeedback
QAxisVoltageCommandValue
```

结构体已经提供上下文，不需要把上下文重复塞进变量名：

```c
M->Sensor.Current.Ia
M->Sensor.Encoder.Theta_m
M->Current.Iq_Ref
M->Motion.Speed_Loop.Sig.Out
```

而不是：

```c
M->MotorSensorData.MeasuredMotorPhaseACurrentAmpere
```

### 2.3 常用后缀

只在确实需要区分语义时使用：

```text
_Ref    参考值
_Fbk    反馈值
_Err    误差
_Est    估计值
_Flt    滤波值
_Raw    原始值
_Cmd    命令
_Lim    限制
_Off    偏置
_Cnt    计数
_Pu     标幺值
_Q31    Q31表示
```

不要为了完整性给所有变量机械添加后缀。

### 2.4 单位

项目内部应统一物理单位。只有同一个物理量存在多种单位表示时，才在名字中加入单位。

推荐内部约定：

```text
Angle       rad
Speed       rad/s
Current     A
Voltage     V
Resistance  Ohm
Inductance  H
Flux        Wb
Time        s
```

如果只有一种表示：

```c
float Wm;
```

不写：

```c
float Mechanical_Angular_Velocity_Rad_Per_Second;
```

---

## 3. 结构体与对象风格

### 3.1 使用静态实例表达对象

同一套代码操作多个实例：

```c
Motor_T PMSM_1;
Motor_T PMSM_2;
```

不要复制第二套控制代码。

### 3.2 算法对象采用 Para / Sig / State

对于 PID、滤波器、观测器、斜坡等有内部状态的模块：

```c
typedef struct
{
    struct
    {
        float Kp;
        float Ki;
        float Kd;
        float Out_Max;
        float Out_Min;
    } Para;

    struct
    {
        float Ref;
        float Fbk;
        float Err;
        float Out;
    } Sig;

    struct
    {
        float Int;
        float Fbk_Pre;
    } State;
} PID_T;
```

含义：

- `Para`：人工配置、标定、保存或调试的参数。
- `Sig`：模块真正需要暴露的输入输出信号。
- `State`：下一周期必须继续使用的历史状态。

### 3.3 临时量必须优先使用局部变量

推荐：

```c
void PID_Run(PID_T *Pid, float Ts)
{
    float P;
    float D;
    float Out;
    float Ki_Ts;

    Ki_Ts = Pid->Para.Ki * Ts;
    ...
}
```

不要因为调试方便或结构看起来完整就保存：

```c
Pid->Coef.Ki_Ts
Pid->Temp.P
Pid->Temp.D
Pid->Calc.Out_Pre
```

只有跨周期需要、外部确实需要观察、或有明确接口意义的数据才进入对象。

### 3.4 逐级组合，不复制全局状态

```c
typedef struct
{
    PID_T Id_Loop;
    PID_T Iq_Loop;
} Current_Ctrl_T;

typedef struct
{
    PID_T Position_Loop;
    PID_T Speed_Loop;
} Motion_Ctrl_T;

typedef struct
{
    Motor_Para_T    Para;
    Motor_Status_T  Status;
    Motor_Cmd_T     Cmd;
    Motor_Sensor_T  Sensor;
    Current_Ctrl_T  Current;
    Motion_Ctrl_T   Motion;
    Motor_Proc_T    Proc;
    Motor_Fault_T   Fault;
} Motor_T;
```

所有可变状态应属于具体实例，避免模块内部隐藏的可变全局变量或 `static` 状态。

---

## 4. 函数设计

### 4.1 函数必须有独立、稳定的语义

推荐：

```c
PID_Run()
Motor_Enable()
Motor_Disable()
Motor_Stop()
Motor_Power_On_Check()
Enc_Cal_Run()
Motor_Proc_Run()
```

这些函数表示完整算法动作或系统动作。

### 4.2 不建立无语义生命周期函数

默认禁止为了形式统一增加：

```c
PID_Init()
PID_Reset()
PID_Clear()
PID_Start()
PID_Stop()
PID_DeInit()
```

如果 `Init()` / `Reset()` 只是把几个字段置零，应直接在具有真实场景含义的位置赋值。

例如进入伺服使能时：

```c
M->Motion.Speed_Loop.State.Int = 0.0f;
M->Current.Iq_Loop.State.Int = 0.0f;
```

而不是：

```c
PID_Reset(&M->Motion.Speed_Loop);
PID_Reset(&M->Current.Iq_Loop);
```

原因：状态属于算法对象，但**状态生命周期策略属于使用它的上层系统**。

### 4.3 Init 只有在真的有初始化工作时才存在

允许 `Init()` 的情况包括：

- 初始化硬件外设；
- 建立 DMA / 队列 / 缓冲区；
- 根据配置生成不能简单静态表达的数据；
- 验证参数并建立必要不变量；
- 资源申请或必须执行一次的构造过程。

单纯零初始化不是建立 `Init()` 的理由。静态 / 全局对象本身由 C 启动过程零初始化。

### 4.4 不为一两行赋值制造包装函数

不推荐：

```c
static void PID_Clear_Int(PID_T *Pid)
{
    Pid->State.Int = 0.0f;
}
```

除非这个动作在系统中已经形成明确且稳定的领域语义。

### 4.5 避免企业软件式名称

除非确实承担该职责，否则避免：

```text
Manager
Service
Interface
Provider
Context
Factory
Dispatcher
Coordinator
Wrapper
Adapter
```

优先使用实际控制功能命名：

```text
Motor_Ctrl
Motor_State
Motor_Fault
Enc_Cal
Phase_Find
RL_Id
Data_Cap
```

---

## 5. 数据所有权

### 5.1 一个物理量只保留必要的一份

不要因为架构习惯同时建立：

```c
Wm_Target
Wm_Cmd
Wm_Ref
Wm_Final_Ref
```

如果不存在真实的处理层，只保留：

```c
Wm_Ref
```

如果确实存在 Ramp：

```c
Speed_Ramp.Sig.In
Speed_Ramp.Sig.Out
```

输入输出归 Ramp 对象，不再在 `Motor_Cmd_T` 复制同样的 Target / Ref。

### 5.2 不保存硬件状态副本

如果硬件可以直接读取：

```c
TIM1->BDTR & TIM_BDTR_MOE
Gate Fault GPIO
TIM Break Flag
```

不要无必要再保存：

```c
Pwm_Active
Gate_Active
Bridge_Active
```

否则软件状态可能与真实硬件不一致。

### 5.3 一个字段尽量只有一个主要写入者

例如：

```text
ADC ISR      -> 电流反馈、电流环状态、快速故障
Motion Task  -> 速度/位置环状态、最终Iq参考
Motor Task   -> 流程状态、慢速故障、伺服状态
Comm/App     -> 正常控制命令
```

跨上下文共享时优先保证写入责任清楚，而不是先建立消息总线或复杂同步框架。

---

## 6. 控制模式与伺服状态

### 6.1 保持最少状态

```c
typedef enum
{
    Servo_Init,
    Servo_Disabled,
    Servo_Enabled,
    Servo_Fault
} Servo_State_E;

typedef enum
{
    Ctrl_Torque,
    Ctrl_Speed,
    Ctrl_Position
} Ctrl_Mode_E;
```

不要把可以由其他量推导的概念继续保存成状态：

```text
Motion_Hold
Motion_Run
Motion_Stopping
Bridge_Active
Bridge_Armed
```

### 6.2 Stop 不等于 Disable

`Stop` 只修改当前模式的参考，不关闭 PWM，不改变 `Servo_Enabled`：

```c
switch (M->Status.Mode)
{
    case Ctrl_Torque:
        M->Cmd.Iq_Ref = 0.0f;
        break;

    case Ctrl_Speed:
        M->Cmd.Wm_Ref = 0.0f;
        break;

    case Ctrl_Position:
        M->Cmd.Theta_Ref = M->Sensor.Encoder.Theta_m;
        break;
}
```

语义：

```text
Torque Stop   -> 零转矩，不位置保持
Speed Stop    -> 零速控制
Position Stop -> 固定位置参考
```

`Disable` 才关闭 MOE / Gate，使电机不再主动输出。

---

## 7. 高频控制代码

ADC / PWM 高频路径不追求形式上的模块化。

允许：

```c
void ADC1_2_IRQHandler(void)
{
    Motor_T *M = &PMSM_1;

    float Ia;
    float Ib;
    float Id;
    float Iq;
    float Ud;
    float Uq;

    /* 直接读取ADC */
    /* Clarke / Park */
    /* 电流环公式 */
    /* SVPWM */
    /* 直接写TIM */
}
```

不要为了“分层正确”强制变成：

```text
ADC_Driver_Read()
Current_Sensor_Service_Process()
FOC_Manager_Run()
Current_Controller_Interface_Update()
PWM_Output_Service_Write()
```

热路径的边界主要由：

- 数据所有权；
- 周期时序；
- 可维护性；
- 执行时间；

决定，而不是函数层数。

---

## 8. 校准、辨识和特殊流程

特殊流程应生成控制要求和推进物理步骤，不建立第二套电机控制系统。

推荐：

```text
Motor_Task
    -> Enc_Cal_Run / Motor_Id_Run
       -> 更新 Proc 参数、参考、Step

Motion_Task
    -> 运行正常速度/位置控制或使用 Proc 参考

ADC ISR
    -> 执行真正的电流/电压控制和PWM
```

流程函数不应：

```text
直接循环调用电流环
直接写PWM寄存器
阻塞等待电机旋转
复制正常控制代码
```

流程内部只保留真正的物理步骤和跨周期数据。

---

## 9. 文件拆分

按**算法族或真实功能**拆文件，而不是按结构体层次拆。

推荐：

```text
PID.c / PID.h
PDFF.c / PDFF.h
LADRC.c / LADRC.h
Motor_Ctrl.c / Motor_Ctrl.h
Motor_State.c / Motor_State.h
Enc_Cal.c
Motor_Id.c
```

不推荐：

```text
PID_Para.h
PID_State.h
PID_Signal.h
PID_Init.c
PID_Reset.c
```

也不要因为一个函数稍长，就机械拆成大量只调用一次的小函数。

---

## 10. 注释风格

注释用于解释**为什么**，不是翻译代码。

推荐：

```c
/* D项作用于反馈，避免参考阶跃产生微分冲击。 */
```

```c
/* 此处使用机械角，电角度在ADC周期内根据极对数换算。 */
```

不推荐：

```c
/* Calculate error */
Err = Ref - Fbk;

/* Set output to zero */
Out = 0.0f;
```

需要统一说明的内容集中到模块头部或设计文档：

- 单位；
- 正方向；
- 电流符号；
- 坐标系；
- 角度范围；
- ADC/PWM时序；
- 特殊算法假设。

---

## 11. Codex 修改代码时的约束

修改本工程时必须遵守：

1. 先阅读相关代码，理解现有信号流和执行周期，再修改。
2. 只修改完成当前任务所需的范围，不顺手做无关重构。
3. 不主动引入新的生命周期函数、状态机、包装器、Manager/Service层。
4. 新增结构字段前，先判断它是否：
   - 跨周期需要；
   - 外部需要读取；
   - 无法由现有数据直接推导。
   三者均不满足时优先使用局部变量。
5. 新增函数前，先判断它是否有独立且稳定的系统/算法语义。只有减少一两行重复代码不足以成为新函数。
6. 不复制一个物理量的多份 Target / Cmd / Ref，除非每一份对应真实存在的处理阶段。
7. 不复制硬件寄存器能够直接表达的状态。
8. 名称优先使用控制工程和物理量符号，避免纯软件式长名称。
9. 高频 ISR 中优先保持执行路径直接，不为了抽象增加不必要函数调用。
10. 若现有代码与本规范冲突，修改当前任务相关代码时逐步向本规范靠拢，但不要扩大重构范围。

---

## 12. 新增抽象前的判断

Codex 在增加任何新的函数、结构体、状态、接口层之前，应先检查：

```text
它是否解决当前已经存在的问题？
它是否表达了一个真实且稳定的控制/系统概念？
去掉它以后，代码是否反而更直接、更容易理解？
它是否只是在包装一两行赋值？
它是否只是为了接口对称？
它是否保存了可以直接计算或读取的数据？
```

如果新增内容主要来自“以后可能有用”“结构看起来完整”“通常软件都这么写”，则不要增加。

---

## 13. 风格示例

### 推荐

```c
void Motor_Stop(Motor_T *M)
{
    switch (M->Status.Mode)
    {
        case Ctrl_Torque:
            M->Cmd.Iq_Ref = 0.0f;
            break;

        case Ctrl_Speed:
            M->Cmd.Wm_Ref = 0.0f;
            break;

        case Ctrl_Position:
            M->Cmd.Theta_Ref = M->Sensor.Encoder.Theta_m;
            break;
    }
}
```

它有明确的伺服系统语义。

### 不推荐

```c
void MotorCommandManagerStopCurrentOperation(
    MotorControlContext_t *MotorControlContext)
{
    MotorReferenceManagerResetTargetReference(
        &MotorControlContext->ReferenceManager);
}
```

问题不是代码能不能运行，而是软件抽象遮住了实际控制含义。

---

## 14. 一句话原则

> **让 C 代码尽量像控制框图的文字表达：数据对应信号，结构对应模块，函数对应真实动作；不用软件架构形式去遮盖本来很简单的控制关系。**
