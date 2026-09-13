# 串口与无线命令参考

`car_firmware` 的 USART1 和 nRF24L01+ 共用同一套文本命令解析器。命令名称
区分大小写，参数之间使用一个或多个空格。

## 传输方式

### USART1

| 项目 | 设置 |
| --- | --- |
| TX / RX | PA15 / PA10 |
| 格式 | 115200, 8-N-1 |
| 流控 | 无 |
| 接收 | DMA receive-to-idle |
| 单次缓冲 | 128 字节 |
| 实际入命令队列 | 最多取前 32 字节 |

串口工具应启用发送行结束符（LF 或 CRLF 均可用于命令）。建议每次只发
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
| `anglebias` | `<degrees>` | 设置最低腿高 44.5 mm 时的俯仰偏置基准 |
| `control` | `off` / `on` | 暂停轮子平衡控制 / 恢复原有稳定启动流程，不保存开关状态 |

推荐遥控帧：

```text
R 0.0 -0.0 0.0 61.5
```

`R` 的字段顺序固定：

1. `turn` → `Differ_Target`，目标左右 RPM 差；
2. `velocity` → `Velocity_Target`，目标平均 RPM；
3. `roll` → `Roll_Target`，单位为度；
4. `height` → `Target_height`，单位为毫米。

`tele_firmware` 会将摇杆速度取反后编码到第二个字段。只在一端修改符号会导致
前进/后退方向反转。

`legheight` 和 `R` 接收时先将共同腿高限制到 `44.5..78.5 mm`，控制任务生成
左右腿目标后分别再次限幅。`Servo angel` 返回限幅后的运动学诊断值。
运动参数和 `R` / `VandD` 的数值输入必须完整、有限；NaN、无穷大、尾随垃圾
和多余参数会被拒绝，不会部分更新一条命令。

### 最低腿高的重心标定

`anglebias` 设置的是 `Angle_bias_min`，始终表示两腿均为最低高度
`44.5 mm` 时的俯仰偏置基准，单位为度。编译默认值为 `9.5°`，运行时修改持续有效。
执行 `save` 后复位自动恢复保存值；没有有效 Flash 记录时使用编译默认值。
NaN 和无穷大输入不会修改基准。

通过遥控器串口发送：

```text
nrfsend anglebias 13.6
```

直接连接小车串口时发送 `anglebias 13.6`。无论当前腿高是多少，这条命令
都把最低腿高的基准设为 `13.6°`。遥控 `R` 帧更新腿高等目标，不修改这个基准。

控制任务每 10 ms 在姿态 PID 运算前计算实时 `Angle_bias`：

```text
h = (限幅后的左腿目标 + 限幅后的右腿目标) / 2
f(h) = 0.01026 * h * h - 1.258 * h + 48.24
Angle_bias = Angle_bias_min + f(h) - f(44.5)
```

以下是指定基准的计算示例（当前编译默认基准为 `9.5°`）：

| 平均腿高 | 基准 12.6° 时的实时偏置 | 基准 13.6° 时的实时偏置 |
| --- | ---: | ---: |
| 44.5 mm | 12.6000° | 13.6000° |
| 61.5 mm | 9.7025° | 10.7025° |
| 78.5 mm | 12.7353° | 13.7353° |

改变基准会整体平移原补偿曲线；升降腿部时基准保持不变，改变的是实时偏置。
“重心”在此指 `Pitch + Angle_bias` 中的角度偏置；腿高采用限幅后的控制目标，
并非传感器测得的实际腿高。算法回归检查及板上验证步骤见
[标定测试说明](../tests/README.md)。

## PID 命令

四组控制参数使用统一格式，`legpid` 是 `rollpid` 的同义命令：

```text
<name> -p <value>
<name> -i <value>
<name> -d <value>
```

| 命令名 | 控制环 | 默认 `Kp / Ki / Kd` |
| --- | --- | --- |
| `anglepid` | 俯仰姿态到共同 PWM | `75.35 / 0 / 60`（Kp 为中间腿高基准） |
| `velocitypid` | 平均轮速到俯仰目标 | `0.05 / 0.008 / 0` |
| `differpid` | 左右轮速差到差速 PWM | `2 / 0.001 / 0` |
| `rollpid` / `legpid` | 横滚到左右腿高度差 | `0 / -0.4 / 0` |

示例：

```text
anglepid -d 55
velocitypid -p 0.04
differpid -i 0.0008
```

`anglepid -p` 修改 `Angle_kp_mid`，含义固定为 **61.5 mm 中间腿高的 Kp**。
实际控制值仍每 10 ms 随左右腿平均高度线性变化：

```text
h = (限幅后的左腿目标 + 限幅后的右腿目标) / 2
Angle_kp = Angle_kp_mid + 0.3 * (h - 61.5)
```

| 平均腿高 | 默认基准 75.35 | 设置 `anglepid -p 80` 后 |
| --- | ---: | ---: |
| 44.5 mm | 70.25 | 74.90 |
| 61.5 mm | 75.35 | 80.00 |
| 78.5 mm | 80.45 | 85.10 |

默认基准恢复原来的 `0.3*h + 56.9` 曲线，正常控制行为保持不变。
修改基准会整体平移直线；升降腿、横滚造成左右腿不等高，都不会覆盖该基准。
Flash 保存基准，绝不保存某个腿高临时算出的 `Angle_kp`。

只有姿态 Kp 和重心实时偏置在现有控制器中随腿高调度。
姿态 Ki/Kd、速度、差速及横滚/腿部 PID 系数保持设定值；它们的实际控制效果仍可能
随机构高度变化，因此需要实车复验。没有机械动力学依据时，不额外给其他增益套线性公式。
`simulation/Leg_kinematics.py` 与固件使用相同连杆尺寸和装配支路；该文件描述的是运动学，
不足以单独推导全部 PID 随腿高的动力学增益。重心沿用原二次补偿曲线。

`rollpid -p` 已修复为修改比例项，`-i` 修改积分项，新增 `-d` 修改微分项。
`legpid` 直接操作同一组参数；当前腿部通过舵机内置位置控制与逆运动学定位，
固件没有另一组独立的“腿高 PID”。原横滚几何补偿、机械尺寸及舵机装配偏置保持原值。

## 保存和查看参数

| 命令 | 作用 |
| --- | --- |
| `save` / `save all` | 一次保存全部可持久化运动参数 |
| `params` | 输出当前基准、有效值、目标、Flash 有效性、未保存状态和控制使能状态 |
| `save recycle` | 日志写满时擦除参数扇区，再保存当前整组参数；未满时等同普通保存 |

保存范围共 15 个浮点数：

- 最低腿高重心基准 `anglebias`；
- `anglepid`、`velocitypid`、`differpid`、`rollpid`/`legpid` 的 P/I/D；
- 当前限幅后的共同腿高 `legheight` 与横滚目标 `target_roll`，作为下次上电姿态目标。

`R` 中的腿高和横滚字段操作相同目标，所以也会进入这次保存快照。
速度、转向、`VandD` 和 `motor` 输出是即时行驶指令，不保存，复位后速度/转向仍为零。
`control off/on`、观测开关、无线遥测槽和 PID 积分/微分历史也不保存。
开机在创建控制任务之前恢复整组配置；第一次使用、格式不兼容或没有完整有效记录时使用编译默认值。

可以在平衡运行时调参，但实际写 Flash 时要求 `Control_armed=false` 且左右 PWM 为零。
静止平衡也属于已使能状态，`save` 会返回 `busy`，且不会延迟到以后自动执行。
STM32F411 写/擦 Flash 时取指和数据读取会停顿，依据
[ST RM0383 §3.5](https://www.st.com/resource/en/reference_manual/dm00119316.pdf)，
保存期间禁止重新使能轮子；结束后重新计满正常启动所需的 500 ms。

直接连接小车串口时，每条命令单独发送并等待返回。例如：

```text
anglebias 10.5
anglepid -p 80
velocitypid -p 0.04
rollpid -i -0.3
params
```

调参完成后，**先扶稳或支撑车体**，再发送：

```text
control off
params
save
params
control on
```

`control off` 在下一次控制更新关闭轮子输出；确认 `params` 中 `armed=false` 后再 `save`。
它不会停止舵机位置保持。`control on` 恢复原来的姿态、角速度及摇杆回中检查。
保存成功返回 `save: ok (all motion parameters)`；与上次保存完全相同则返回
`save: unchanged (no flash write)`。Flash 错误不会撤销 RAM 中已调好的参数。

通过遥控器串口发送时，小车命令加 `nrfsend` 前缀：

```text
nrfsend anglepid -p 80
nrfsend control off
nrfsend save
nrfsend params
nrfsend control on
```

命令之间等待处理完成。若需要保存指定腿高/横滚，先在遥控器执行 `joystick off`，
再显式发送所需 `R` 帧，避免自动摇杆帧覆盖姿态目标；`joystick off` 本身不会清零目标。
遥控器的 `nRF: send success` 是无线 ACK，**不是 Flash 保存成功回执**。
本次命令结果和 `params` 多行文本仍输出到小车 USART1；可经现有 UART 透传蓝牙模块读取，
没有新增原生 BLE 协议，也没有改变原来的 nRF 遥测格式。

`params` 的 `p_mid` 表示持久化基准，`p_effective` 是控制任务最近一次计算值；
刚修改参数后最多等一个控制周期再核对有效值。`flash_valid` 表示存在完整保存记录，
`unsaved` 表示当前参数与该记录不同（没有记录时也为 true）。后续 `R` 帧改变姿态目标
会使 `unsaved` 再次变为 true，但不会改掉已保存的 Flash 内容。

### Flash 寿命、断电与更新固件

链接脚本给程序保留前 384 KB，扇区 7（`0x08060000..0x0807FFFF`，128 KB）
专用于参数，不放入生成的 ELF/HEX/BIN 镜像。每条记录 84 字节，可追加 1560 条；
仅手动保存且参数有变化时写入，调参和遥控帧不会自动写 Flash。

记录带版本、字段数、序号、CRC32，写入并回读验证数据后，最后写提交标记。
启动扫描最新的有效记录，跳过写到一半或校验失败的记录。普通保存被中断时，
上一条完整记录仍保留；首次保存尚未完成时使用编译默认值。

写满后普通 `save` 返回 `full` 并保留旧记录。此时可在供电稳定、控制未使能时
执行 `save recycle`。这是单扇区存储的显式维护操作：**擦除到重新提交之间断电，
可能丢失全部历史保存值，下次上电回到编译默认值**。维护前用 `params` 留存参数。

按项目默认的 ELF/HEX 范围烧录可以避开参数扇区；全片擦除、手工擦除扇区 7、
使用旧链接布局的大镜像或启用下载器的全片擦除选项都会清掉参数。
CubeMX 重新生成后必须保留两个链接脚本的 384 KB 程序限制和参数区符号。
修改存储字段顺序或语义时必须升级 `MotionParameterJournal.hpp` 的 `version`，
并明确决定迁移或回退默认值，不能直接把旧记录解释成新参数。

## 观测和诊断

| 命令 | 作用 |
| --- | --- |
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
中断延迟和 UART 丢帧，只用于短时诊断。

nRF 遥测默认四个槽为 Roll、Pitch、Yaw、Angle `Kp`，约每 100 ms 发送
一次。车端发送期间不处于接收模式；发送成功或达到最大重试次数后才切回 RX。
遥控闭环运行时不建议开启周期遥测。

## 兼容/实验命令

```text
motor <left> <right>
```

解析成功后会打印数值并通知 MotionControl。当前实际运行的
`MotionControlFunc_PID()` 没有消费这条通知，因此该命令不会覆盖闭环 PWM；
它只保留用于旧的 LQR/手动电机实验。

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

确认参数后，按上述步骤执行 `save`，不需要修改源码重新烧录。
编译默认重心基准位于 `Component/UserApp/CtrlAlgorithm/BalanceCompensation.hpp` 的
`default_minimum_bias_degrees`，其他可保存参数的默认值位于 `Component/UserApp/MotionParameters.hpp`。
已有有效 Flash 记录时，修改编译默认值不会覆盖保存的参数。

