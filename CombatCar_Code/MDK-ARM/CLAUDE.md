# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指导。

## 项目概览

对抗车固件，基于 STM32H743VITx (Cortex-M7, 双精度 FPU)。一台差速驱动机器人，在平台上自主导航：巡逻、通过激光雷达索敌追踪、攻击、掉台后重新上台。

## 构建方式

- **IDE**: Keil MDK-ARM 5 (uVision)。项目文件：[`duikang_che.uvprojx`](duikang_che.uvprojx)
- **编译器**: ARM Compiler 5 (V5.06 update 7)，不是 GCC/Clang——使用 `__CC_ARM` 预定义，`int`/`long` 均为 32 位，使用 Keil 专有内联函数
- **SDK**: STM32H7xx HAL，通过 `Keil.STM32H7xx_DFP.4.1.3` pack 引入。输出 `.hex` 文件位于 `duikang_che/duikang_che.hex`
- **没有命令行构建方式**，只能通过 uVision GUI 编译。`.clangd` 仅用于 IDE 代码智能提示（指向 ARMCC 头文件 `D:\Program Files\Keil_v5\ARM\ARMCC\include`）

## 源码布局

MDK-ARM 目录存放 Keil 工程文件，实际源码在上层 `CombatCar_Code/` 目录树中：

- `Core/Src/` — HAL 层：`main.c`（入口、时钟初始化）、`freertos.c`（任务创建）、`stm32h7xx_it.c`（中断处理）、外设初始化（`gpio.c`、`dma.c`、`tim.c`、`usart.c`）
- `Core/Inc/` — `FreeRTOSConfig.h`、`stm32h7xx_hal_conf.h`、`main.h`（引脚定义）
- `Drivers/STM32H7xx_HAL_Driver/` — STM32 HAL 库（标准库，不要修改）
- `Middlewares/Third_Party/FreeRTOS/` — FreeRTOS 内核 + CMSIS-RTOS V2 封装
- `User/` — **所有应用代码均在此目录**

## RTOS 任务架构

共 4 个任务，在 [`Core/Src/freertos.c`](../Core/Src/freertos.c) 中创建：

| 任务 | 优先级 | 栈大小 | 职责 |
|---|---|---|---|
| `SensorTask` | Normal | 2048B | 响应 UART 中断通知，轮询处理所有传感器数据 |
| `MainTask` | Normal | 2048B | 主状态机——行为决策、电机控制 |
| `StageTask` | High | 2048B | 平台边角检测（激光雷达 + 小激光传感器融合） |
| `DangerTask` | High | 512B | GPIO 边缘传感器——触发紧急脱困 |

**传感器通知模式**：UART 中断服务函数（`Task_Uart.c`）将数据路由到对应驱动，然后通过 `vTaskNotifyGiveFromISR` 通知 `SensorTask`。`SensorTask` 每次被唤醒后处理所有排队中的 DMA/缓冲区数据。电机指令由 `MainTask` 直接调用，不经过队列——`MainTask` 是电机输出的唯一写入者。

## 主状态机

`CarState` 枚举定义在 [`User/Task_Main.h`](../User/Task_Main.h)，流转如下：

```
INIT → PATROL → TRACK → ATTACK
                   ↑        │
                   └────────┘
                   ↓
              AVOID_FRIENDLY_BLOCK （AprilTag 检测到己方能量块）
              EDGE_ESCAPE （DangerTask 通过 GPIO 边缘传感器触发）
              OFF_STAGE_SEARCH → REENTER_STAGE → PATROL （由 StageTask 的 g_stage_type 驱动）
```

通过 `TaskMain_SwitchState()` 切换状态。每个状态在 `Task_Main_Run()` 中以 1ms 为周期循环执行。

## 传感器系统

| 传感器 | 驱动文件 | 串口 | 通信方式 |
|---|---|---|---|
| JY901S IMU | `User/JY901S.c` | USART1 | RX 中断，提供偏航/俯仰/横滚角 |
| 4路小激光测距 | `User/LaserRange.c` | UART4/5/7/8 | DMA 接收，按方向索引取距离值 |
| 扫描激光雷达 | `User/LiDAR.c` | USART2 (DMA) | 360°扫描，360个点，1°分辨率 |
| AprilTag 视觉 | `User/Vision.c` | USART3 | RX 中断，识别己方能量块 ID |

## 平台区域检测（`Task_Stage.c`）

将激光雷达 360° 扫描数据与 4 个方向的小激光测距融合，判断车相对于平台的位置：

1. **上下差检测**：每个主方向计算 `差值 = 雷达远处距离 - 小激光近处距离`。差值大说明该方向靠近平台边缘（小激光能打到台面，雷达能打到远处墙壁）
2. **角区检测**：将靠近墙壁的雷达点分割为直线段，然后在前角区域检测是否存在正交直角边
3. **平台区域类型**：`STAGE_ON`（台上）、`STAGE_OFF_CANDIDATE`/`STAGE_OFF_UNKNOWN`（台下）、`STAGE_MIDDLE_{FRONT,BACK,RIGHT,LEFT}`（平行于边缘）、`STAGE_CORNER_{FRONT,RIGHT/LEFT,REAR,RIGHT/LEFT}`（在角落）

结果写入 `volatile StageType g_stage_type`，由 `MainTask` 读取使用。

## 索敌模块（`Track_Enemy.c`）

每周期处理一整圈激光雷达快照数据：先修补扫描数据中的零点空洞，屏蔽已知遮挡区域（后方 ±128°-180°、前角 ±39°-49°），然后基于距离连续性聚类将剩余数据点分割为候选目标段。输出 `EnemyInfo`（角度、距离、点数），由 `MainTask` 的 TRACK/ATTACK 状态使用。

## 电机控制（`Motor.c`）

差速驱动，通过 TIM1 输出 4 路 PWM（每轮 2 路，用于 H 桥方向控制）。`Motor_SetSpeeds(left, right)` 将速度钳制在 ±900 PWM（90% 占空比），施加 ±20 死区，然后将正负号映射到对应通道。`MOTOR_DEBUG_FORCE_STOP` 编译时开关可彻底禁用电机输出。

## 引脚定义

电机 PWM：PE9/PE11（左轮）、PE13/PE14（右轮）。边缘传感器：PD6（右）、PD7（左）——低电平有效（`GPIO_PIN_RESET` = 危险）。

## 代码风格

- C99，4 空格缩进，Linux 风格大括号（见 [`.clang-format`](.clang-format)）
- 无符号整数字面量加 `U` 后缀
- 函数使用 snake_case，类型使用 PascalCase，模块函数加前缀（如 `TrackEnemy_`、`TaskStage_`）
- 文件内部辅助函数加 `static`；跨文件全局状态通过 `extern` 暴露（如 `state_manager`、`g_stage_type`）
- clangd 仅用于 IDE 辅助，不用于编译诊断（`.clangd` 第 76 行抑制了所有诊断）

## 调试开关

- 将 [`User/Motor.h`](../User/Motor.h) 中的 `MOTOR_DEBUG_FORCE_STOP` 设为 1，彻底禁用电机输出
- 将 [`User/Task_Main.c`](../User/Task_Main.c) 中的 `DEBUG_AUTO_TEAM` 设为 1，跳过台上靠墙测距判断阵营的步骤，便于桌面调试
- [`User/Task_Stage.c`](../User/Task_Stage.c) 中的 `StageDebugInfo g_stage_debug_info` 暴露了平台检测的所有中间值，可在调试器中直接查看
