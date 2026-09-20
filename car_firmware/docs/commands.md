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

## 腿部舵机安装模式

最低腿高安装不能用 `control off` + `legheight` 代替：前者关闭舵机 PWM，后者仍受平衡和横滚补偿影响。
使用以下独立命令（USART1/BLE 与可选 nRF 共用）：

| 命令 | 行为 |
| --- | --- |
| `install on` | 立即屏蔽双轮 PWM，清空速度/转向/横滚目标，双腿运动至当前运动学的最低腿高 44.5 mm 并保持 |
| `install status` 或 `install` | 查询安装开关与舵机目标到位状态 |
| `install off` | 退出安装、停止舵机 PWM，车端控制保持关闭；需要运行时另发 `control on` |

BLE 推荐发送 `@install on\n`、`@install status\n`、`@install off\n`。
这些短命令也兼容不带 `@` 的单包行，小程序使用不带前缀的短行。

```text
install: active=1 ready=1 height=44.5 control=off
```

- `active=1` 时，轮电机输出始终为零，自适应腿高/横滚补偿停止，`R`、`VandD`、`target_roll`、`legheight` 和 `control on` 被拒绝。
- 舵机仍通过现有的运动学、左右映射和偏置驱动。44.5 mm 对应逻辑舵机角约 1.189°，当前两路 PWM 比较值分别为 513、2365（1 µs/count）。
- `ready=1` 表示软件平滑目标已到位，舵机没有位置反馈；开始装配前目视确认静止。进入时可能先回复 `ready=0`，稍后查询。
- 切换小程序界面、蓝牙断开、遥控超时和 `control off` 不退出安装模式。必须显式 `install off`，或重启设备；安装开关不写入 Flash。
- 退出后不恢复旧遥控目标；恢复 IMU 采样时重新初始化姿态融合。`control on` 后重新经过至少 500 ms 的正常稳定姿态门控。
- 只在系统 `ready` 且无参数存储操作时允许进入；硬件初始化失败、运行故障仍遵循原安全门控。
- 安装期间暂停 IMU 采样，`status` 的 `imu=0` 表示没有新的姿态样本，并非陀螺仪初始化失败。

## 控制命令

上电和复位默认使用原有普通启动，需显式 `coldstart start` 才执行
[支架冷启动发车](cold-start.md)，双腿发车目标为 69.5 mm。`controlstate` 报告
`coldstart=<阶段> abort=<原因码> travel=<毫米> gain=<0..1>`，用于查看发车进度。
`control off`、`install on` 或独占 Flash 维护会取消本次发车；之后 `control on`
使用普通 500 ms 平衡启动门控，不会自动前进。重新发车仍需显式 `coldstart start`。
冷启动进行期间，运动目标由发车流程接管，普通遥控目标仍接收并遵循 500 ms 超时；
约 12 cm 后恢复当时有效的目标。停止发车可用 `coldstart stop` 或 `control off`。

| 命令 | 作用 |
| --- | --- |
| `coldstart start` | 开启控制并重新执行完整发车；正在发车时重复请求保持当前进度。正常平衡输出期间须先停控，安装/故障/存储忙时拒绝 |
| `coldstart stop` | 关闭车端控制、立即停止输出并取消待处理发车请求 |
| `coldstart status` / `coldstart` | 回执 `coldstart: phase=... active=0/1 abort=0..6 travel=<mm> gain=<0..1> control=on/off` |
| `autoleg on` / `autoleg off` | 开关横滚自适应腿高；关闭时两腿同高，重开渐进恢复。仅运行时生效，默认开启 |
| `autoleg status` / `autoleg` | 回执 `autoleg: enabled=0/1 active=0/1`；前者为用户设置，后者为当前是否实际调节 |

这些命令均支持 USART1/BLE 和可选 nRF。小程序使用单包短行 `coldstart start\n`、
`autoleg off\n` 等，串口分包发送使用 `@` 前缀。冷启动前进目标为 −30 RPM，
与遥控器/小程序的前进方向一致。详见 [冷启动](cold-start.md#手动发车与自适应腿高)。

| 命令 | 参数 | 作用 |
| --- | --- | --- |
| `R` | `<turn> <velocity> <roll> <height>` | 一次更新转向、速度、横滚和腿高目标 |
| `VandD` | `<difference> <velocity>` | 更新左右轮速差和平均速度目标 |
| `target_roll` | `<degrees>` | 更新横滚目标 |
| `legheight` | `<millimetres>` | 更新共同腿高目标，并打印运动学计算结果 |
| `anglebias` | `<degrees>` | 设置最小腿高 44.5 mm 的俯仰重心基准；无参数查询基准和实际补偿值 |
| `deadzone` | `<0..1000>` | 同时设置左右轮的最小非零 PWM compare；无参数查询当前值，0 关闭补偿 |

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

`deadzone` 无有效 Flash 记录时默认 0，即不启用死区补偿。设置后在下一次有效 10 ms 电机输出周期
同时应用到 TB6612 A（左轮）与 B（右轮）：控制器算出的 PWM 为 0 时仍保持 compare
为 0；非零绝对值小于死区时抬升到死区值。数值大于 1000、负数、小数、尾随字符
或多余参数均拒绝，保留上一次有效值。该参数会直接改变电机起转力度，调节前应架空
车轮并保留硬件断电手段。

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

姿态参数和电机死区在下一次有效 10 ms 控制循环生效，速度、差速、横滚参数在下一次
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
| `save` / `save all` | 平衡运行中保存全部运动参数到 Flash，无需 control off；相同值不重复写入 |
| `params` | 查询参数、`flash_valid`、`unsaved`、`armed`、`enabled` 及补偿后的实际值 |
| `control off` | 请求关闭轮平衡；用于手动停机及扇区回收前准备，不是普通 save 的前置步骤 |
| `control on` | 系统 ready 时恢复正常启动门控，重新稳定 500 ms 才解锁；不能解除故障锁存 |
| `save recycle` | 日志写满后的维护：需未解锁且 PWM 为零，必要时擦除扇区并保存 |

普通 `control off` 在系统 ready 时继续 IMU 采样和姿态融合，输出保持关闭；
再次 `control on` 不清空已估计的陀螺零偏。安装模式和独占维护暂停采样的路径仍会
重新初始化融合。持续 IMU 读取失败仍会锁存故障，不能用 `control on` 绕过。

保存范围为重心基准、电机输出死区、角度/速度/差速/横滚四组 PID 的 P/I/D、
共同腿高、横滚目标，共 15 个浮点数，加上角度 Kp 自动/固定模式和一个 0..1000
的整数死区值。`legpid` 是 `rollpid` 的别名。
开机只读取 Flash 并恢复到 RAM；遥控调参只修改 RAM，控制任务读取 RAM。
只有明确收到 save/save all/save recycle 才执行写入，写入内容为命令处理时的整组快照。
未保存的修改在重启或断电后丢失，没有有效 Flash 记录时使用编译默认值。
速度和转向指令、遥控超时计时、控制开关、PID 历史、诊断与遥测开关不保存。
上电恢复保存的姿态目标，但新的遥控帧仍会更新它们；遥控超时保护继续生效。

小程序的“保存全部参数”按钮发送 ASCII **`@save\n`**（最后一个字节为 LF `0x0A`）。
按行缓冲接收回包，允许 BLE 分片，再按以下前缀判断结果：

| 回包前缀 | 小程序处理 |
| --- | --- |
| `save: ok` | 保存成功 |
| `save: unchanged` | 参数已保存，无需重复写入；按成功处理 |
| `save: busy` | 系统启动中、已有保存操作，或运行中请求了 recycle；普通 save 不因平衡已解锁而被拒绝 |
| `save: full` | 提示日志已满；另设带说明的回收操作，发送 `@save recycle\n` |
| `save: invalid` / `save: flash error` | 保存失败，显示错误；RAM 调参值仍保留 |

普通按钮只发 `@save\n`，不发送 control off/on，也不自动执行 save recycle。
普通保存不改变平衡解锁状态、运动目标或 PID 历史，只有显式回收才要求停止控制。
命令应与周期遥控帧串行发送，避免字节交织。nRF 也接受相同命令正文，但执行结果
当前输出到小车 USART1；遥控器的串口调参桥接白名单未开放 `save`，不能用无线 ACK
代替保存成功确认。

参数使用 STM32 内部 Flash 扇区 7（`0x08060000..0x0807FFFF`），固件限制在前
384 KiB。每条记录 84 字节，含版本、CRC32 和最后写入的提交标记；最多 1560 条。
普通保存逐字追加记录，每写一个 32 位字就让出一个系统 tick，让高优先级平衡任务继续
执行，不关闭控制或重置稳定门控。只有 CRC 与最终提交标记回读确认后才返回 ok。
途中掉电仍可恢复前一条有效数据；第一条记录尚未完成时则回退默认值。v3 继续保持
v1/v2 的槽大小，将死区写入版本字高 16 位，并兼容读取旧记录；v1/v2 记录的死区
解释为编译默认值 0，v1 自动解释为中间腿高基准模式。`save recycle` 擦除窗口内掉电没有第二扇区
备份，会回退编译默认值。整片擦除也会清除参数；正常只擦写固件占用扇区可保留参数。

首次安装或其他固件转入使用 `flash_factory`（清除扇区 7，写入默认记录）；日常更新
使用 `flash_update`（保留扇区 7）。factory 已占用第一条记录，首次未调参的 `save`
返回 unchanged。镜像和完整操作见 [烧录说明](flash-images.md)。

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
