# 小车端软件架构

本文描述 `car_firmware` 当前实际运行的任务、数据流、控制环和并发约束。
硬件接线、构建与首次上电流程见项目根目录的 [README](../README.md)。

## 数据流

```mermaid
flowchart LR
    Remote["遥控器"] -->|32-byte ASCII payload| NRF["nRF24L01+<br/>SPI2 + DMA"]
    UART["USART1<br/>Receive-to-idle DMA"] --> Reactor["LEDBlink / Reactor task"]
    NRF --> Reactor
    Reactor --> Targets["共享目标和 PID 参数"]

    IMU["MPU6050<br/>I2C1"] --> Motion["MotionControl task"]
    Encoders["TIM2 / TIM3<br/>quadrature encoders"] --> Motion
    Targets --> Motion
    Motion --> Motor["TB6612<br/>TIM1 PWM"]
    Motion -->|50 ms notification| ServoTask["ServoControl task"]
    ServoTask --> Kinematics["四连杆逆运动学"]
    Kinematics --> Servos["TIM9 CH1 / CH2"]

    Motion --> UART
    Reactor --> UART
```

命令接收和控制计算通过 `AppModules` 内部的 `ControlState` 连接。字段保持
`volatile` 以兼容原有 ISR/任务访问方式；扩展跨任务的多字段事务时，仍需增加
快照或明确的同步机制，不能把 `volatile` 当作互斥量。

## 启动流程

```mermaid
sequenceDiagram
    participant Reset as Reset
    participant C as Core/Src/main.c
    participant RTOS as FreeRTOS
    participant CPP as CPP_Main()
    participant Manager as AppManager
    participant Tasks as Application tasks

    Reset->>C: startup_stm32f411ceux.s
    C->>C: HAL_Init + 100 MHz clock
    C->>C: GPIO, DMA, UART, I2C, TIM, RTC, SPI
    C->>RTOS: osKernelInitialize()
    C->>RTOS: MX_FREERTOS_Init()
    RTOS->>CPP: create defaultTask, then call CPP_Main()
    CPP->>Manager: register communication, servo, motion
    Manager->>Tasks: create tasks in dependency order
    alt any task creation fails
        Manager->>Tasks: delete created tasks in reverse order
    end
    C->>RTOS: osKernelStart()
    RTOS->>Tasks: schedule tasks
```

`CPP_Main()` 在调度器启动前创建任务。默认 CMSIS-RTOS2 任务只等待 1 s 后
删除自身，不参与控制。`CPPMain: success` 表示三个模块均完成注册且三个任务的
`xTaskCreate()` 全部返回 `pdPASS`；失败时管理器按反序回滚已创建任务。

## 任务模型

FreeRTOS tick 为 1 kHz，`xTaskCreate()` 的栈深度单位是 32-bit word。

| 任务 | 优先级 | 栈 | 周期/唤醒源 | 主要职责 |
| --- | ---: | ---: | --- | --- |
| MotionControl | 29 | 2500 words | 10 ms | IMU、编码器、串级 PID、电机 PWM |
| LEDBlink | 28 | 2000 words | UART/nRF 通知；100 ms 超时 | 命令解析、无线 IRQ、遥测、PC13 |
| ServoControl | 28 | 256 words | MotionControl 每 50 ms 通知 | 腿高限幅、逆运动学、舵机目标 |
| defaultTask | CMSIS low | 64 words | 启动后等待 1 s | 删除自身 |

`LEDBlink` 的名称来自早期实验。它现在是 UART/nRF 事件反应器，LED 翻转只是
其中一个周期动作。

### MotionControl

实际运行的是 `AppModules` 中的 PID 控制任务。每 10 ms：

1. 根据腿高重新计算俯仰静态偏置和姿态环 `Kp`；
2. 读取 MPU6050，经 VQF 得到 Roll、Pitch、Yaw；
3. 更新姿态 PID；
4. 组合直行和差速 PWM，限幅后写入 TB6612。

MPU6050 初始化失败时，控制任务把两个电机 PWM 保持为 0，并以 1 s 周期让出 CPU；
不会在缺少姿态反馈时继续运行平衡控制，串口/无线诊断任务仍可调度。

每累计 5 次，即每 50 ms：

1. 读取左右轮 RPM；
2. 速度 PID 根据平均 RPM 生成俯仰目标；
3. 差速 PID根据左右 RPM 差生成转向 PWM；
4. 横滚增量式 PID 和几何补偿生成左右腿高度差；
5. 通知 `ServoControl` 更新舵机。

控制关系可概括为：

```text
angle_target = velocity_pid(velocity_target, average_rpm)
differ_pwm   = differ_pid(differ_target, left_rpm - right_rpm)
even_pwm     = angle_pid(angle_target, pitch + angle_bias)

left_pwm  = clamp(even_pwm + differ_pwm, -1000, 1000)
right_pwm = clamp(even_pwm - differ_pwm, -1000, 1000)
```

横滚误差绝对值超过 3° 时，会叠加正弦几何补偿。左右腿目标随后被限制到
`44.5..78.5 mm`。

### ServoControl

任务阻塞等待直接任务通知，不主动轮询。收到通知后：

1. 限制左右目标腿高；
2. 通过 `LegKinematics::getMotorAngleForHeight()` 求舵机角；
3. 两侧都减去 10° 装配偏置；
4. 调用 `setAngle_Smooth(..., 1000)` 更新平滑目标。

`Servo` 使用 10 ms FreeRTOS 软件定时器逐步逼近目标。

### LEDBlink / Reactor

`TaskReactor` 给每个信号分配一个任务通知 bit：

- UART receive-to-idle 完成后，将最多 32 字节放入命令队列；
- nRF IRQ 到来后读取状态；收到 payload 时将其放入同一个命令队列；
- TX 成功或达到最大重试次数后，将 nRF 切回 RX；
- 命令队列容量为 4；
- 100 ms 无事件超时时翻转 PC13；启用遥测时同时发送一帧 nRF 数据。

命令名称区分大小写。解析器按空格分隔 token，不提供转义、校验和或身份认证。

## 当前默认控制参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| Angle `Kp / Ki / Kd` | `70 / 0 / 60` | 姿态环；运行时 `Kp` 会随腿高重算 |
| Angle bias | `12.6°` | 运行时会随腿高重算 |
| Velocity `Kp / Ki / Kd` | `0.05 / 0.008 / 0` | 平均轮速到俯仰目标 |
| Difference `Kp / Ki / Kd` | `2 / 0.001 / 0` | 左右轮速差到差速 PWM |
| Roll `Kp / Ki` | `0 / -0.4` | 横滚到左右腿高度差 |
| Velocity target | `0` | RPM 目标 |
| Difference target | `0` | 左右 RPM 差目标 |
| Roll target | `0°` | 车体横滚目标 |
| Leg height | `44.5 mm` | 共同腿高目标 |
| Motor dead zone | `50 / 50` | TB6612 A/B PWM counts |

在线修改方式见 [命令参考](commands.md)。

## 中断与 DMA

| 中断 | NVIC 优先级 | ISR 到任务的同步 |
| --- | ---: | --- |
| SPI2 RX/TX DMA | 5 | binary semaphore |
| USART1 RX/TX DMA | 5 | RX 通知 / TX 固定缓冲队列 |
| SPI2 global | 5 | HAL SPI IRQ |
| USART1 global | 5 | HAL UART IRQ |
| nRF IRQ / EXTI15_10 | 6 | direct-to-task notification |
| TIM2 / TIM3 | 6 | HAL TIM IRQ |
| TIM5 HAL time base | 15 | HAL tick |

`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 为 5。调用 FreeRTOS ISR API
的中断，NVIC 数值优先级不能小于 5。

周期等待、I2C 读取、格式化和 PWM 更新均不在 FreeRTOS 临界区中。尤其禁止把
`vTaskDelayUntil()` 或其他阻塞 API 放进临界区，否则 tick/PendSV 无法调度，
控制任务可能锁死系统。多字段共享状态若需要一致快照，应只在短临界区内复制。

## 内存模型

- FreeRTOS heap 为 32 KiB，使用 `heap_4.c`；
- 应用任务通过 `xTaskCreate()` 动态创建，但只在启动阶段分配；
- nRF SPI 同步信号量通过 `xSemaphoreCreateBinary()` 创建；
- UART 数据缓冲、ETL 队列和命令表使用固定容量；
- UART 默认有 10 个 128 字节 TX 缓冲和 10 个 128 字节 RX 缓冲；
- UART TX 缓冲耗尽时，新日志会被静默丢弃；
- 控制周期内不应新增 `new`、`malloc` 或无界 STL 容器。

## 模块边界

| 模块 | 当前职责 |
| --- | --- |
| `MPU6050` / `vqf` | I2C 采样、量程换算、姿态融合 |
| `HallEncoder` | 定时器正交编码、累计计数、RPM 换算 |
| `TB6612` | PWM、方向和死区 |
| `Servo` | 物理角度到脉宽、软件定时器平滑 |
| `LegKinematics` | 四连杆正/逆运动学 |
| `PID` | 位置式和增量式 PID |
| `NRF24L01P` | SPI 寄存器、收发状态机、IRQ 处理 |
| `LkUart` | Receive-to-idle DMA 和异步格式化发送 |
| `TaskReactor` | 通知 bit 到回调的分发、命令 token 解析 |
| `Component/App` | 固定容量模块注册、生命周期、失败回滚 |
| `Component/AppModules` | 通信、舵机、运动控制任务和私有运行状态 |
| `Component/UserApp/main.cpp` | 只负责注册和启动应用模块 |

`LQR` 是保留的实验控制器；当前构建会编译它，但应用模块不启动该路径。
`MainControl.*` 目前为空壳，不在运行路径中。

## 已知实现约束

以下是阅读代码或调参时必须知道的当前行为：

- `Angle_kp` 和 `Angle_bias` 每 10 ms 根据腿高重算，在线写入只会短暂生效；
- 当前平均腿高表达式实际为 `(Left_Legheight + Left_Legheight) / 2`，没有读取
  右腿高度；若依赖左右腿平均值，应先修正并重新标定；
- `rollpid -p` 与 `rollpid -i` 分别写入横滚 `Kp` 和 `Ki`；
- `motor` 命令会发送任务通知，但 PID 控制路径没有消费该通知；
- TB6612 的零速度命令保持 compare 为 0；只有非零且低于死区的命令才提升到
  50 counts；
- nRF 遥测命令会临时把车端从 RX 切到 TX，发送完成或失败后才恢复 RX；
- 控制参数只保存在 RAM 中，复位后恢复编译时默认值。
