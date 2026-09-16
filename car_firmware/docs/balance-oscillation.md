# 中点震荡：固件修改与实车对比

本次修改针对“能平衡，但在中点反复摆动/抖动”。代码检查能确认下列问题，
但没有实车波形，不能据此认定电机质量差，也不能证明震荡已经消除。

## 已处理的问题

| 原行为 | 新行为与目的 |
| --- | --- |
| MPU6050 的 ±1000°/s 换算写成 `30768/1000`，角速度约大了 6.5% | 改为 `32768/1000`，驱动测试覆盖实际寄存器到角速度换算 |
| DLPF=0、分频却按 1 kHz 计算；高频振动直接进入 100 Hz 读取 | DLPF=3，陀螺约 42 Hz、加速度约 44 Hz，分频基准回到 1 kHz，输出 100 Hz |
| 对 `Pitch + angle_bias` 差分，腿高/标定变化也触发微分推力 | 使用 VQF 去零偏后的陀螺速度，按横滚姿态投影到俯仰轴；D 不再响应补偿值阶跃 |
| 50 ms 轮速的单个计数直接进入速度/差速环 | 25 ms 时间常数的一阶低通，保留直流响应，减小交替跳变；编码器按实际间隔计算 RPM |
| I 只限制累计误差，输出饱和时仍可能积累 | 位置式 PID 使用条件积分，允许反向退积分；轮 PWM 饱和时暂停外环积分 |
| 设死区后，任意非零微小请求都直接输出最小推力 | 请求绝对值 0..40 内逐渐引入补偿，减少过零正反冲击；0 保持 0 |
| 调度超时后可能连续补跑控制周期 | 超时后从当前 tick 重新排期，避免对同一传感器状态反复积分 |

IMU 配置依据 [InvenSense MPU-6000/6050 寄存器说明，寄存器 25/26](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)。
条件积分用于输出饱和后的恢复，原理可参见 [MathWorks 的 anti-windup 示例](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html)。

没有给姿态误差增加“停止控制区”，也没有对整个平衡 PWM 加慢速斜坡。
姿态比例反馈继续每 10 ms 更新。陀螺 D 保留原参数尺度：
`D = -angle.kd * pitch_rate_deg_s * 0.01`；例如 Kd=60、俯仰角速度 +50°/s 时为 -30 PWM。
轮速滤波会增加约 25 ms 的低频延迟，因此外环仍需要实车确认，不能保证原先所有调参都最优。

默认 PID、重心基准、腿高和 Flash 格式保持原值。已经保存的参数会继续覆盖编译默认值；
`deadzone` 的数值不变，但零点附近含义改为渐进补偿。补偿默认仍为 0，实际静摩擦未知时
不自动猜测一个最小推力。零点变软可能让某些电机起转变慢，需结合下述步骤验证。

## 先做可比较的实车试验

记录 `params` 回包，再烧录 `build/balance-Release/WL1_F411CEU6.elf`，保留参数扇区 7。
本次产物使用默认 USART1 9600 波特率、nRF 关闭；如原车依赖 nRF 或不同串口速率，
按 README 用 `WL1_ENABLE_NRF24` / `WL1_COMMAND_UART_BAUD` 重新构建。
先用低腿高和可扶住车体的平地环境测试，出现明显加速或倾倒时立即停止。
改变参数前后保持相同电量、地面和腿高，一次只改一项，暂不 `save`。

1. 先保留现有 PID，对比新旧固件。如果以前设置了很大的死区，先记录原值，
   以 `@deadzone 0\n` 做基线，观察是否仍有同样震荡。
2. 若表现为车身慢速来回摆，临时发送 `@velocitypid -i 0\n`，短时观察。
   速度积分关闭后允许小幅漂移，因此这一步只用于区分积分引起的来回纠偏。
   如果明显改善，再从较小 I 开始增加；例如原 I=0.008 时可先试 0.002，再以 0.001 小步增加。
   若没有改善，恢复记录值，继续检查姿态增益和机械问题。
3. 若表现为快速发抖，用 `balancediag` 看 `rate` 和 `request` 是否随之快速变号。
   先将姿态 P 降低约 10% 做对比；例如原基准 P=75.35，可试 `@anglepid -p 68\n`。
   不要同时改 P、D 和死区。此例是实验起点，不是已经实测的最优值。
4. 若 PWM 已缓慢增大，轮子却长时间不转、随后突然跳动，可试逐步增加
   `deadzone`（例如 0、20、40，每次观察后再决定）。如果来回冲击加重则退回。
   新曲线在请求绝对值达到 40 后才施加完整死区值。
5. 稳定后用小扰动检查恢复，再测试遥控回中和腿高变化；确认后 `@save\n`。
   未保存的调参可通过重新上电恢复此前记录。

以上示例中的 `\n` 表示真正的换行字节。`@control off\n` 可停轮，
再次 `@control on\n` 后仍要满足连续 500 ms 的正常启动门控。

## 记录什么

直接连接小车 UART/BLE，停止 `showimu/showrpm/nrfshow` 连续输出，轮询：

```text
@controlstate
@balancediag
```

两条合计每秒一组即可进行初步检查，避免占满默认 9600 波特率串口。
它们是按需快照，无法还原快速震荡的频谱；进一步分析需更高带宽的记录方式。
`controlstate` 的 `pwm` 现在是补偿后的实际有符号 compare；正负号仍为车体控制方向，
右轮的接线反相由驱动完成。

`balancediag` 示例格式：

```text
rate=0.00 rpm=0.00,0.00 filtered=0.00,0.00
tilt=0.000 request=1,1 pwm=2,2
```

- `rate`：俯仰角速度，°/s，D 项使用它。
- `rpm`：左右编码器原始窗口速度；`filtered`：滤波后的平均速度、左右速度差。
- `tilt`：速度环给姿态环的目标，°；来回大幅变化说明外环正在来回纠偏。
- `request`：死区补偿前的有符号 PWM；`pwm`：驱动实际施加的 PWM。
- 只有 `armed=1`、`imu=1` 时以上数据代表有效闭环；停机/读取失败会清控制历史。

如果软件改动后仍有同样的“轮子卡住然后跳动”，再断电检查齿轮间隙、轮轴阻力、
左右电机一致性、IMU 安装松动及电机供电。这些物理非线性需要实测，固件不能完全消除。

## 软件验证与边界

2026-09-16：主机 Debug / Release 各 50 项 CTest 全部通过；STM32 Debug / Release
均成功生成 ELF、HEX、BIN。Release 占用 Flash 262,968 B / 384 KiB、RAM 48,832 B / 128 KiB，
参数扇区占用为 0。未烧录或进行实车负载验证。

回归包含真实 MPU6050 寄存器驱动/VQF 零偏、真实 TB6612 输出和编码器回绕、
过零补偿单调性、姿态 D 的符号/尺度、标定与腿高阶跃无 D 冲击、轮速噪声衰减、
PID 抗饱和/实际周期、停机重启、遥控超时以及 Flash 保存互锁。

```sh
cmake -S tests -B build/host-balance-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-balance-debug --parallel
ctest --test-dir build/host-balance-debug --output-on-failure
cmake -S . -B build/balance-Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/balance-Release --parallel
```

这些检查验证软件行为，不模拟完整车体/电机动力学，也不代表完成了带负载实车验证。
