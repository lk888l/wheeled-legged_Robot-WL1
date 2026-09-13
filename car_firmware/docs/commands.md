# 串口与无线命令参考

`car_firmware` 的 USART1 和 nRF24L01+ 共用同一套文本命令解析器。命令名称
区分大小写，参数之间可以使用空格或制表符。数值 token 必须完整，拒绝 NaN、
Inf、溢出和数字后的杂字符；多字段命令全部解析成功后才一起发布目标。

控制必需硬件初始化失败进入安全模式后，只要 USART1 或 nRF 至少一条命令通道初始化成功，
命令服务仍会运行。参数写入和诊断应答会保留，但平衡任务不会运行，执行器输出
请求会被安全门控拒绝。

## 传输方式

### USART1

| 项目 | 设置 |
| --- | --- |
| TX / RX | PA15 / PA10 |
| 格式 | 默认 9600, 8-N-1（ZX-D30）；CMake `WL1_COMMAND_UART_BAUD` 可配置 |
| 流控 | 无 |
| 接收 | DMA receive-to-idle |
| 单次缓冲 | 128 字节 |
| 实际入命令队列 | 不超过 32 字节；超长帧整帧拒绝 |

蓝牙客户端推荐发送 `@<命令>\n`（例如 `@anglepid -p 75\n`）：支持任意分包、
LF/CRLF、同一接收块内多条命令；命令正文最多 32 字节。`@` 为重新同步标记。
旧格式仍兼容“一次 UART 空闲事件对应一条完整命令”，但没有标记的命令不能跨
空闲事件拼接。不要混用旧协议与分包。详见 [蓝牙串口](bluetooth-uart.md)。

### nRF24L01+

默认跳过 nRF 初始化。用 `-DWL1_ENABLE_NRF24=ON` 恢复可选无线通道。
无线 payload 固定为 32 字节。有效 ASCII 文本之后应补 `0x00`。车端收到
payload 后，在任务上下文中使用与串口相同的解析器执行。

命令队列深度为 4。连续突发超过处理能力时可能丢失命令，因此遥控器应保持
稳定周期，不要一次发送多条拼接命令。

## 控制命令

| 命令 | 参数 | 作用 |
| --- | --- | --- |
| `R` | `<turn> <velocity> <roll> <height>` | 一次更新转向、速度、横滚和腿高目标 |
| `VandD` | `<difference> <velocity>` | 更新左右轮速差和平均速度目标 |
| `target_roll` | `<degrees>` | 更新横滚目标 |
| `legheight` | `<millimetres>` | 更新共同腿高目标，并打印运动学计算结果 |
| `anglebias` | `<degrees>` | 设置最小腿高 44.5 mm 的俯仰重心基准；无参数查询基准和实际补偿值 |

推荐遥控帧：

```text
R 0.0 -0.0 0.0 61.5
```

`R` 的字段顺序固定：

1. `turn` → `ControlParameters::difference_target`，目标左右 RPM 差；
2. `velocity` → `velocity_target`，目标平均 RPM；
3. `roll` → `roll_target`，单位为度；
4. `height` → `leg_height`，单位为毫米。

`tele_firmware` 会将摇杆速度取反后编码到第二个字段。只在一端修改符号会导致
前进/后退方向反转。

`legheight` 输入经横滚补偿后，由运动任务将两腿目标限制到 `44.5..78.5 mm`。命令返回的
`Servo angel` 诊断值是在限幅前计算的，因此越界输入只适合检查算法，不代表
舵机实际会到达该位置。

`anglebias` 与 `main` 保持相同含义：设置 44.5 mm 腿高的基准角，当前默认 7.0°。
运动任务使用已限幅的左右腿目标平均值 `h` 计算：

```text
effective_bias = base_bias + (h - 44.5) * (0.01026 * (h + 44.5) - 1.258)
```

例如 `anglebias 10.5` 后，44.5 mm 时实际 bias 为 10.5°，61.5 mm 时约为
7.60252°。变更腿高或接收后续 `R` 帧不会覆盖这个基准。这里使用的是目标腿高，
没有实际腿高传感器反馈。输入 `anglebias` 可查询两种数值。

## PID 命令

四组 PID 使用统一格式，USART1 与 nRF 接收路径均支持：

```text
<name> -p <value>
<name> -i <value>
<name> -d <value>
```

| 命令名 | 控制环 | 默认 `Kp / Ki / Kd` |
| --- | --- | --- |
| `anglepid` | 俯仰姿态到共同 PWM | `70 / 0 / 60` |
| `velocitypid` | 平均轮速到俯仰目标 | `0.05 / 0.008 / 0` |
| `differpid` | 左右轮速差到差速 PWM | `2 / 0.001 / 0` |
| `rollpid` | 横滚到左右腿高度差 | `0 / -0.4 / 0` |

示例：

```text
anglepid -d 55
velocitypid -p 0.04
differpid -i 0.0008
```

上电默认保留自动姿态增益 `Kp = 0.3 * average_leg_height + 56.9`，与 `main`
一致：44.5 mm 时为 70.25，61.5 mm 时为 75.35。`anglepid -p 80` 会切换到
手动 Kp，并持续使用 80；后续 `R`、`legheight` 或横滚补偿不会覆盖它。
`anglepid -auto` 恢复按平均腿高计算 Kp。`-i`、`-d` 不改变 Kp 模式。

姿态参数在下一次有效 10 ms 控制循环生效，速度、差速、横滚参数在下一次
50 ms 外环生效。未解锁时轮 PWM 保持 0，设置值仍保留。所有参数只保存在 RAM，
复位后恢复默认值。

输入 `anglepid`、`velocitypid`、`differpid`、`rollpid`（无参数）查询当前参数。
`anglepid` 还返回 `mode=auto/manual` 和上次控制循环使用的 `effective_p`。

横滚控制提供：

```text
rollpid -p <value>
rollpid -i <value>
rollpid -d <value>
```

三项分别更新对应横滚参数；仅启用 Kd 时也参与增量式 PID 计算。

## 观测和诊断

| 命令 | 作用 |
| --- | --- |
| `ping` | 返回 `pong`、当前状态和控制开关，检查命令服务存活 |
| `status` | 返回状态、控制开关、硬件失败位图和任务失败位图 |
| `controlstate` | 返回平衡解锁、IMU 有效性、补偿后俯仰、左右 PWM、循环次数及采样间隔 |
| `button` | 返回 PA0 任务状态、click/double/long 计数、丢事件数和最大扫描间隔 |
| `showimu -y` | 以约 100 Hz 输出 `Roll,Pitch,Yaw` |
| `showimu -n` | 停止 IMU 连续输出 |
| `showrpm -y` | 以约 20 Hz 输出左右轮 RPM |
| `showrpm -n` | 停止 RPM 连续输出 |
| `nrfsend <text>` | 立即发送一帧原始文本 payload |
| `nrfshow -mr <slot>` | 将遥测槽 `0..3` 映射到 Roll 并开始发送 |
| `nrfshow -mp <slot>` | 将遥测槽 `0..3` 映射到 Pitch 并开始发送 |
| `nrfshow -my <slot>` | 将遥测槽 `0..3` 映射到 Yaw 并开始发送 |
| `nrfshow -nn` | 停止周期 nRF 遥测 |

`showimu` 会在 10 ms 控制环内格式化并提交 UART 日志。长时间开启可能增加
控制计算耗时和 UART 丢帧，只用于短时诊断；格式化已移出临界区。

nRF 遥测默认四个槽为 Roll、Pitch、Yaw、Angle `Kp`，约每 100 ms 发送
一次。车端发送期间不处于接收模式；发送成功或达到最大重试次数后才切回 RX。
遥控闭环运行时不建议开启周期遥测。

`status` 示例：

```text
status=init-failed control=off hw_fail=130 task_fail=0
```

硬件位从 bit 0 起依次表示 USART1 命令接收、MPU6050、左编码器、右编码器、
轮电机 PWM、左舵机、右舵机、nRF24L01+。任务位从 bit 0 起依次表示
Heartbeat、CommandService、ServoControl、MotionControl、ButtonA0。ButtonA0
是可选业务任务，其失败 bit 4 不阻止控制任务运行。按键接入见 [PA0 按键](button-a0.md)。

`control=on` 表示硬件/任务允许控制；`armed=1` 才表示轮平衡已解锁。启动需连续
50 个 10 ms 有效样本满足：补偿后俯仰绝对值 ≤8°、横滚 ≤5°、角速度 ≤20°/s、
速度/转向目标绝对值 <1。俯仰或横滚超过 30°、IMU 读取失败或非有限值立即取消
解锁并清除 PID 历史。连续三次 IMU 异常进入锁存的 `runtime-fault`。

`controlstate` 的 `gap` 是历史最大采样间隔（tick，当前 1 tick=1 ms），`missed`
统计间隔超过 10 tick 的次数；这些统计会受调试器暂停影响，不代表纯计算耗时或包含暂停时长的墙上时间。

## 兼容/实验命令

```text
motor <left> <right>
```

当前 PID 路径不支持原始 PWM 点动，明确返回
`motor rejected: raw PWM is unavailable in PID control mode`。
旧版本只打印并发送一个无人消费的通知；现在不再给出已接受的假象，也不绕过闭环。

未知命令不会返回 `Unknown command`，而是按以下格式回显：

```text
receive: <original text>
```

## 调参建议

调参前架空车轮，并保留硬件断电手段。一次只修改一个参数：

1. 确认 IMU Pitch 正方向和电机纠偏方向正确；
2. 关闭速度和差速目标，调整姿态环；
3. 调整速度环，使平均 RPM 平稳跟踪；
4. 调整差速环，使左右轮速差跟踪转向目标；
5. 最后调整横滚和腿高补偿；
6. 在不同腿高、供电电压和地面摩擦条件下复验。

当前参数只存在 RAM 中。确认参数后，应修改
`Component/UserApp/ControlState.hpp` 中的默认初始化值，重新构建并烧录。

