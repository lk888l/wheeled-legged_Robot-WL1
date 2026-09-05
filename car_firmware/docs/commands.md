# 串口与无线命令参考

`car_firmware` 的 USART1 和 nRF24L01+ 共用同一套文本命令解析器。命令名称
区分大小写，参数之间可以使用空格或制表符。数值 token 必须完整，拒绝 NaN、
Inf、溢出和数字后的杂字符；多字段命令全部解析成功后才一起发布目标。

初始化失败进入安全模式后，只要 USART1 或 nRF 至少一条命令通道初始化成功，
命令服务仍会运行。参数写入和诊断应答会保留，但平衡任务不会运行，执行器输出
请求会被安全门控拒绝。

## 传输方式

### USART1

| 项目 | 设置 |
| --- | --- |
| TX / RX | PA15 / PA10 |
| 格式 | 115200, 8-N-1 |
| 流控 | 无 |
| 接收 | DMA receive-to-idle |
| 单次缓冲 | 128 字节 |
| 实际入命令队列 | 不超过 32 字节；超长帧整帧拒绝 |

串口工具应启用发送行结束符（LF 或 CRLF 均可用于带参数命令）。建议每次只发
一条短于 32 字节的命令，并等待其处理完成。

### nRF24L01+

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
| `anglebias` | `<degrees>` | 写入俯仰静态偏置 |

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

`anglebias` 仍接受存储值，但当前控制模式每 10 ms 自动按腿高校准 bias，
不使用人工写入值；需要人工模式时应先实现模式切换。

## PID 命令

三组常规 PID 使用统一格式：

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

示例：

```text
anglepid -d 55
velocitypid -p 0.04
differpid -i 0.0008
```

angle Kp 会每 10 ms 按校准腿高重算为 `0.3 * height + 56.9`，因此
`anglepid -p` 的存储值不参与当前自动校准。Angle `Ki`、`Kd` 和速度/差速参数会持续到
下次复位。

横滚控制提供：

```text
rollpid -p <value>
rollpid -i <value>
```

`-p` 更新横滚 Kp，`-i` 更新横滚 Ki；已修复旧实现将两者都写入 Ki 的问题。

## 观测和诊断

| 命令 | 作用 |
| --- | --- |
| `ping` | 返回 `pong`、当前状态和控制开关，检查命令服务存活 |
| `status` | 返回状态、控制开关、硬件失败位图和任务失败位图 |
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

