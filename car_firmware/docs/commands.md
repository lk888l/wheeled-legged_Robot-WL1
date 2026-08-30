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
| `R` | `<turn> <velocity> <roll> <height> [profile [flags]]` | 原子更新遥控目标和可选档位/动作位 |
| `VandD` | `<difference> <velocity>` | 更新左右轮速差和平均速度目标 |
| `target_roll` | `<degrees>` | 更新横滚目标 |
| `legheight` | `<millimetres>` | 更新共同腿高目标，并打印运动学计算结果 |
| `anglebias` | `<degrees>` | 写入俯仰静态偏置 |

推荐遥控帧：

```text
R 0.0 -0.0 0.0 61.5
```

`R` 的字段顺序固定：

1. `turn` → `Differ_Target`，目标左右 RPM 差；
2. `velocity` → `Velocity_Target`，目标平均 RPM；
3. `roll` → `Roll_Target`，单位为度；
4. `height` → `Target_height`，单位为毫米；
5. 可选 `profile`：`0` 柔和、`1` 普通、`2` 运动；
6. 可选 `flags`：bit0 武装跳跃，bit1 发出跳跃请求（只认上升沿）。

四个主字段分别限制为转向/速度 `-100..100`、横滚 `-18..18°`、腿高
`44.5..78.5 mm`。一帧的四个字段在短临界区内同时提交，运动任务不会读到
新旧字段混合值。有效 `R` 帧中断超过 250 ms 后，速度、转向、横滚会归零，
跳跃同时解除武装；姿态平衡内环继续运行。队列会保留 UART/无线来源，串口
测试帧不能伪装成新鲜无线链路；缺少 `flags` 的旧帧不具备跳跃权限。

`tele_firmware` 会将摇杆速度取反后编码到第二个字段。只在一端修改符号会导致
前进/后退方向反转。

`legheight` 输入会先限制到 `44.5..78.5 mm`，再计算并打印运动学结果。

`anglebias <degrees>` 会启用持久手动偏置并限制为 `5..20°`；`anglebias auto`
恢复按腿高自动计算。实际值最多以 `5°/s` 渐变，避免一帧把姿态环推入饱和。

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

`anglepid -p` 现在是持久手动 `Kp` 覆盖；`anglepid auto` 恢复
`0.3 * average_height + 56.9`。其他 PID 参数也会持续到下次复位。为避免实时
命令把控制器直接推入长期饱和，固件会将手动值限制在已知量级：姿态
`P=40..120, I=0..1, D=30..100`，轮速 `P=0..0.15, I=0..0.03,
D=0..0.1`，差速 `P=0..5, I=0..0.01, D=0..1`，横滚
`P=-1..1, I=-1..0`。原始参数不会一帧跳变，而是按约 1 秒跨越完整允许范围的
速率渐变；跳跃已武装、处于动作阶段或 Fault 锁存时，原始 PID 和偏置修改会被
拒绝。跳跃 Ready 稳定计时还会等待全部参数渐变完成，动作期间参数保持冻结。

实时强度档位使用：

```text
pidlevel 0   # 柔和：降低外环增益和速度/转向/横滚权限
pidlevel 1   # 普通：当前稳定参数
pidlevel 2   # 运动：提高外环响应，保持已验证的姿态内环
```

档位在 400 ms 内插值，切换时会预置 PID 历史，避免微分冲击。柔和档仍保留
完整的 `±1000` 平衡 PWM 权限。

## 跳跃预留命令

```text
jump status
jump arm
jump trigger
jump disarm
```

状态机包含 Ready、Preload、Thrust、Flight、Landing、Recover 和 Fault，使用
IMU 加速度、Roll 和角速度监督阶段及超时。因为舵机没有位置反馈，它只是受 IMU
监督的时间轨迹，不是舵机位置闭环。默认构建
`WL1_ENABLE_EXPERIMENTAL_JUMP=OFF`，上述命令只运行 dry-run，绝不会覆盖真实
腿高；实际执行开关不能通过无线或串口打开。实验构建若在动作中进入 Fault，
解除武装只会先锁存确认；左右软件目标到达 52 mm、舵机任务确认已消费目标且
软件平滑完成，再保持 200 ms 后才清除 Fault。舵机任务失联或控制周期超过
30 ms 会锁存故障。由于没有位置回读，这仍不等于证明舵机轴实际到位。

横滚控制提供：

```text
rollpid -p <value>
rollpid -i <value>
```

`rollpid -p` 写入横滚 `Kp`，`rollpid -i` 写入横滚 `Ki`。两者会持续到下次复位。

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
| `nrfstatus` | 显示无线 ready、IRQ、电源重试、补轮询、收包、SPI 错误和丢队列计数 |
| `controlstatus` | 显示档位、IMU/失联/跳跃状态，以及舵机 ready/完成、目标序号和心跳年龄 |

`showimu` 输出 `Roll,Pitch,Yaw,a=<|a|g>,ok=<accel trusted>`。它会在 10 ms
控制环内格式化并提交 UART 日志。长时间开启可能增加
中断延迟和 UART 丢帧，只用于短时诊断。

nRF 遥测默认四个槽为 Roll、Pitch、Yaw、Angle `Kp`，约每 100 ms 发送
一次。车端发送期间不处于接收模式；发送成功或达到最大重试次数后才切回 RX。
遥控闭环运行时不建议开启周期遥测。

## 兼容/实验命令

```text
motor <left> <right>
```

解析成功后会打印数值并通知 MotionControl。当前实际运行的 PID 控制任务没有
消费这条通知，因此该命令不会覆盖闭环 PWM；
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

当前参数只存在 RAM 中。确认参数后，应修改
`Component/AppModules/src/runtime.hpp` 中的默认初始化值，重新构建并烧录。

