# CAN FD 控制通信

## 1. 当前范围

AxDr 的 Parameter、Action、Event 原本就采用 CAN FD 标准帧 ID 与 DLC/数据字段格式。
本次增加板载 FDCAN1 的实际收发，**不修改应用层协议**。
USB CDC 仍以 `AXDR + ID(2B LE) + Len(1B) + Data` 封装同样的 AxDr 消息。
CAN 总线直接传输 `ID + DLC + Data`，不携带 `AXDR` 字节帧头。

本阶段不在 CAN 上实现 Plot 配置、FAST/NORMAL 连续采集或定时广播；
USB Plot 保持现有行为。未来 CAN 一问一答实时数据查询另行定义。

## 2. 硬件和总线

- AxDrive-L V1.3：U9 TJA1051TK/3/1J，PB8 RX、PB9 TX。
- CN11：**1 脚 CAN_L，2 脚 CAN_H**。板上收发器已焊装，不需外接收发器模块。
- PC13 / R_EN 上电低电平，经 U17 接入 R96 **120 Ω**；本板位于总线端点时保留接入。
- FDCAN1 使用标准 11-bit ID、FD+BRS、标称 **1 Mbps**、数据段 **5 Mbps**。
- `AxDr_L_Motor.ioc` 将 FDCAN 中断优先级设置为 8。当前收发采用 Motor Thread **2 kHz 有界轮询**，并未使能 FDCAN 收发中断通知，也不额外分配 ThreadX 通信线程。
- CAN FD 数据段采用 TDC offset 16，必须通过真实对端的 BRS 通信确认物理裕量。

单条总线原则上仅在两端连接 120 Ω；多节点时不要让每块板都接入终端。

## 3. 协议 ID 与响应路由

`CAN_ID = (Msg_Type << 6) | Node_ID`，当前 `AXDR_NODE_ID = 1`。

| 11-bit ID | 消息 | 方向 |
| --- | --- | --- |
| `0x1C1` | Parameter / Action | 主机 → 电机 |
| `0x081` | Response | 电机 → 请求来源 |
| `0x201` | Event / Action Complete | 电机 → 主机 |

FDCAN 硬件过滤器仅接收本节点的 `0x1C1` 标准 ID 数据帧；拒收其他 ID、扩展帧与远程帧。当前要求使用 CAN FD 帧。

USB 请求的响应回 USB，CAN 请求的响应回 CAN；电机线程处理异步 Action 后也保留其来源。`Protection` 状态变化 Event 分别向 USB 与已启动的 CAN 输出。CAN 与 USB 使用独立响应队列，不互相竞争读取。报文 DLC 必须是 CAN FD 允许的长度，较长响应的填充字节为 0。

### 参数读取示例

读取 `PARAM_MOTOR_MODE = 0x0701`：

```text
CAN FD + BRS, Standard ID: 0x1C1
DLC: 4 bytes
DATA: 01 01 01 07
      |  |  +---+-- Parameter ID = 0x0701, little-endian
      |  +--------- AXDR_PARAM_READ = 0x01
      +------------ Txn = 0x01
```

收到成功响应：CAN ID `0x081`，数据以 `01 07 01 00` 开头（Txn、请求消息类型、操作、成功状态），后接 ID、类型和参数值。

### Action 示例

以 `ACTION_MOTOR_ENABLE = 0x1001` 为例：

```text
CAN FD + BRS, Standard ID: 0x1C1
DLC: 5 bytes
DATA: 02 02 01 10 06
      |  |  +---+-- Action ID = 0x1001, little-endian
      |  |         06 = PARAM_ACTION
      |  +--------- AXDR_PARAM_WRITE = 0x02
      +------------ Txn = 0x02
```

Action 的同步执行响应走 `0x081`。例如辨识和寻相的异步完成结果，额外使用 `0x201` Event 发送 `03 Txn ActionId_L ActionId_H Status`；动作的允许状态、条件和成功失败码与 USB 完全一致。

## 4. 调试与待验证

本次仅完成固件实现与构建准备，**没有以实机 CAN FD 适配器确认收发成功**。
建议按顺序测试：

1. 板卡与支持 CAN FD/BRS 的适配器作为总线两端分别终端匹配，验证 `0x1C1 → 0x081` 参数读取。
2. CAN 写入无危险 RAM 参数并读回；分别执行 Enable、Stop、Disable，检查响应状态；再验证异步 Action Complete。
3. 同时连接 USB，分别请求不同 Txn，确认两条物理链路的响应互不串线；USB FAST/NORMAL Plot 行为不变。
4. 修改保护状态，确认 `0x201` Event；发送错误 ID、扩展 ID、Classic CAN 帧确认不能触发协议执行。
5. 验证无 CAN 适配器、断线、错误帧及重连后的自动重传与 Bus-Off 行为；当前启用自动重传，无对端 ACK 时不能将“入硬件 TX FIFO”视作发送成功。
6. 电机 RUN 时按 1 kHz 请求速率测 CAN 往返延迟，同时检查 `Deadline_Miss`、`Enc_Miss`、Motor Thread 堆积和 `CAN_Error_Cnt`。

`CAN_Rx_Cnt` 为通过基本帧格式检查的收到报文计数，
`CAN_Tx_Cnt` 表示消息已提交硬件 TX FIFO，**不代表获得 ACK**；
`CAN_Error_Cnt` 为 HAL 操作失败计数。可用 SWD 读取这些诊断量。
