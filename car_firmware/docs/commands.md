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

`legheight` 输入先限制到 `44.5..78.5 mm`，诊断角度按限幅后的高度计算；
运动任务再计算横滚补偿并分别限制两腿目标。

`anglebias` 设置 44.5 mm 腿高的基准角，无有效 Flash 记录时默认 9.5°。
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
| `anglepid` | 俯仰姿态到共同 PWM | `75.35 / 0 / 60` |
| `velocitypid` | 平均轮速到俯仰目标 | `0.05 / 0.008 / 0` |
| `differpid` | 左右轮速差到差速 PWM | `2 / 0.001 / 0` |
| `rollpid` / `legpid` | 横滚到左右腿高度差 | `0 / -0.4 / 0` |

示例：

```text
anglepid -d 55
velocitypid -p 0.04
differpid -i 0.0008
```

姿态 P 默认采用 61.5 mm 中间腿高的基准值，实际值按双腿平均目标高度线性补偿：

```text
effective_p = p_mid + 0.3 * (average_leg_height - 61.5)
```

默认 `p_mid=75.35`，仍与原曲线 `0.3*h+56.9` 一致。`anglepid -p 80`
设置基准并启用自动补偿：44.5/61.5/78.5 mm 对应 74.9/80/85.1，腿高变化
不会回写或覆盖基准。需要固定 Kp 时用 `anglepid -manual 80`；用 `anglepid -auto`
重新启用补偿，保持当前基准数值。`-i`、`-d` 不改变模式。模式也随 `save` 保存。

这是与此前模块化版本的语义调整：原先依赖 `anglepid -p` 固定 Kp 的客户端，
应改发 `anglepid -manual <value>`；正常调参按钮继续用 `-p` 调整基准。

姿态参数在下一次有效 10 ms 控制循环生效，速度、差速、横滚参数在下一次
50 ms 外环生效。未解锁时轮 PWM 保持 0，设置值仍保留。修改先在 RAM 生效，执行 `save` 才写入 Flash；
复位或完全断电后恢复最后保存的一组值，没有有效记录才使用编译默认值。

输入 `anglepid`、`velocitypid`、`differpid`、`rollpid`（无参数）查询当前参数。
`anglepid` 还返回 `mode=auto/manual` 和上次控制循环使用的 `effective_p`。

横滚控制提供：

```text
rollpid -p <value>
rollpid -i <value>
rollpid -d <value>
```

三项分别更新对应横滚参数；仅启用 Kd 时也参与增量式 PID 计算。

## Flash 参数保存

| 命令 | 作用 |
| --- | --- |
| `save` / `save all` | 一次保存全部运动参数，断电保留；相同值不重复写 Flash |
| `params` | 查询参数、`flash_valid`、`unsaved`、`armed`、`enabled` 及补偿后的实际值 |
| `control off` | 请求关闭轮平衡；等待 `params` 中 `armed=false` 后保存 |
| `control on` | 系统 ready 时恢复正常启动门控，重新稳定 500 ms 才解锁；不能解除故障锁存 |
| `save recycle` | 日志写满后，显式擦除参数扇区并保存当前值 |

保存范围为重心基准、角度/速度/差速/横滚四组 PID 的 P/I/D、共同腿高、横滚目标，
共 15 个浮点数，加上角度 Kp 自动/固定模式。`legpid` 是 `rollpid` 的别名。
速度和转向指令、遥控超时计时、控制开关、PID 历史、诊断与遥测开关不保存。
上电恢复保存的姿态目标，但新的遥控帧仍会更新它们；遥控超时保护继续生效。

小程序的“保存全部参数”按钮发送 ASCII **`@save\n`**（最后一个字节为 LF `0x0A`）。
按行缓冲接收回包，允许 BLE 分片，再按以下前缀判断结果：

| 回包前缀 | 小程序处理 |
| --- | --- |
| `save: ok` | 保存成功 |
| `save: unchanged` | 参数已保存，无需重复写入；按成功处理 |
| `save: busy` | 当前已解锁、PWM 非零或系统启动中；停止控制后重试，不会排队自动保存 |
| `save: full` | 提示日志已满；另设带说明的回收操作，发送 `@save recycle\n` |
| `save: invalid` / `save: flash error` | 保存失败，显示错误；RAM 调参值仍保留 |

普通按钮不要自动追加 `control on` 或自动执行 `save recycle`。`control off` 是轮平衡
开关，不等于切断所有舵机输出。默认按钮只发送保存请求；遇 busy 再由用户停止控制。
命令应与周期遥控帧串行发送，避免字节交织。nRF 也接受相同命令正文，但执行结果
当前输出到小车 USART1；遥控器的串口调参桥接白名单未开放 `save`，不能用无线 ACK
代替保存成功确认。

参数使用 STM32 内部 Flash 扇区 7（`0x08060000..0x0807FFFF`），固件限制在前
384 KiB。每条记录 84 字节，含版本、CRC32 和最后写入的提交标记；最多 1560 条。
普通保存追加记录，途中掉电仍可恢复前一条有效数据。v2 保持 v1 的槽大小并兼容读取
旧记录；v1 自动解释为中间腿高基准模式。`save recycle` 擦除窗口内掉电没有第二扇区
备份，会回退编译默认值。整片擦除也会清除参数；正常只擦写固件占用扇区可保留参数。

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

确认参数后使用 `save` 保存，无需重新构建或烧录。

