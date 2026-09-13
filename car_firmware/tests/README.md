# 姿态、启动控制、重心标定与参数保存验证

Windows PowerShell 可在 `car_firmware` 目录一次运行全部主机回归：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:/rj/CLion 2024.1/bin/mingw/bin/g++.exe'
```

测试分别使用 `-Og` 和 `-O3 -fno-fast-math`，覆盖：

- VQF 首帧到 10 秒静止输入、四种姿态，包含低通滤波初始化阶段；
- 上电稳定 500 ms 才使能、非中位命令/身体运动阻止启动、倾倒和 IMU 异常退出及恢复；
- 根据可调重心基准判断平衡姿态，PID 首帧/重置后的微分及真实 TB6612 驱动的零 PWM 行为；
- 最低腿高基准、左右腿平均值及补偿曲线；
- 全部运动参数命令、横滚 P/I 分离及 D 支持、61.5 mm 基准增益和反复腿高变化；
- Flash 整组往返、逐字中断恢复、逐字段损坏、格式/CRC/非有限值拒绝、回读错误、日志写满与显式回收；
- 保存期间禁止使能、瞬间保存后重新计满 500 ms，以及 `control off/on`；
- 误用 `-Ofast` 时编译必须被 VQF 保护检查拒绝。

`tests/stubs/tim.h` 仅为主机测试模拟 GPIO 和 PWM 寄存器，不参与小车固件构建。

## 主机算法回归

在 `car_firmware` 目录使用支持 C++23 的主机 C++ 编译器执行（不要使用
`arm-none-eabi-g++` 生成主机测试程序）：

```sh
mkdir -p build/balance-tests
g++ -std=c++23 -Og -g -Wall -Wextra -Werror -I Component/UserApp \
  tests/balance_compensation_test.cpp -o build/balance-tests/balance-debug
./build/balance-tests/balance-debug
g++ -std=c++23 -O3 -fno-fast-math -Wall -Wextra -Werror -I Component/UserApp \
  tests/balance_compensation_test.cpp -o build/balance-tests/balance-release
./build/balance-tests/balance-release
```

Windows PowerShell 使用 `New-Item -ItemType Directory -Force build/balance-tests`
创建目录，将每条编译命令写成一行，并为输出程序添加 `.exe` 后缀。

覆盖最低/中间/最高腿高参考值、基准增量对整条曲线的影响、左右交换对称性、
先分别限幅再平均、重复高度变化及 NaN/无穷大基准检查。优化构建也运行同一组
检查。这些测试不连接硬件，不验证无线命令是否收到。

## 固件构建

按项目 README 分别构建 Debug 与 Release，检查 ELF/HEX/BIN 均成功生成。
上述主机测试不加入 STM32 固件的源码列表。

## 后续板上验证

确认 ST-Link 接在小车主控，按项目的上电检查准备车体，再烧录并校验小车镜像。
用 Debug ELF 观察以下变量：

```gdb
print Angle_bias_min
print Angle_bias
print Left_Legheight
print Right_Legheight
print Angle_kp
print Angle_kp_mid
print Control_armed
print Control_imu_valid
print Control_pitch_error
print Control_left_pwm
print Control_right_pwm
```

无线命令通过遥控器串口发送。需要固定 `R` 帧目标时，先发送 `joystick off`，
再发送 `R 0 0 0 44.5`；这会明确指定零速度、零转向、零横滚和最低腿高。
`joystick off` 本身只暂停自动发包，小车保留上一组目标。

1. 没有有效保存记录时，复位后基准为 `9.5°`、两腿目标初值为 `44.5 mm`；
   否则恢复之前保存的基准和共同腿高。未满足启动条件时两腿
   保持共同高度，`Control_armed` 为假、轮子输出及 TIM1 CCR1/CCR2 为 0。
2. 发送 `nrfsend anglebias 13.6`，运行多个控制周期后确认基准保持 `13.6°`。
3. 两腿目标均为 `44.5 mm` 时，实时偏置应为 `13.6°`；均为 `61.5 mm`
   时应约为 `10.70252°`；均为 `78.5 mm` 时应约为 `13.73532°`。
4. 在中间高度修改基准，再降回最低高度，确认实时偏置回到新基准；重复发送
   `R` 帧后基准仍保持不变。
5. 横滚控制可能让两腿目标不相等，应按观察到的限幅后左右腿平均值核对公式，
   不直接把 `Target_height` 当作平均高度。交换左右高度时，理论偏置应相同。
6. 调试完成后恢复需要的基准和运动目标，再用 `joystick on` 恢复摇杆周期发送。

启动门限见项目 README：回中摇杆、扶到重心补偿后的平衡姿态附近并稳定 500 ms
后进入控制。检查时以 `Control_imu_valid` 和 `Control_armed` 区分有效但不在
平衡范围的姿态与传感器故障。电机/舵机断开的测试只能验证电气输出，不能验证
闭环机构实际纠偏或负载下的平衡性能。

GDB 停机读变量会暂停控制循环，仅在按项目说明准备好的调试状态下使用。
无线 ACK 只证明包已被模块确认，命令是否应用以上述基准和实时值为准。

## 参数保存板上验收

主机测试的 NOR Flash 模型会限制对齐和单向编程，并逐个写入位置注入故障，
但不能替代 STM32 真正的 Flash 操作、UART/nRF 收包或闭环负载测试。
本节为待在实板执行的验收步骤；不要把主机通过解释为已完成烧录或实车测试。

1. 按 README 准备车体并烧录，读取小车 USART1 的 `params` 启动日志。
   检查镜像只占前 384 KB，没有擦除参数扇区 7。
2. 暂停遥控器自动摇杆帧（`joystick off`），扶稳车体后发送 `nrfsend control off`，
   等待小车 `params` 显示 `armed=false`，确认两轮 PWM 为零。
3. 逐条修改 `anglebias`、四组 PID 的 P/I/D、`legheight` 和 `target_roll`，
   通过 `params` 记录整组参数。确认 `rollpid -p` 不改 I，`legpid` 与 `rollpid` 显示同组值。
4. 设置 `anglepid -p 80`，分别使用 `R 0 0 0 44.5`、`R 0 0 0 61.5`、
   `R 0 0 0 78.5`。等待更新后，有效 Kp 分别为 74.9、80、85.1，`p_mid` 始终为 80。
5. 显式设定希望保存的腿高及横滚目标，发送 `nrfsend save`；小车输出应为 `save: ok`。
   再次保存相同参数应输出 `unchanged`，`params` 的 `unsaved=false`。
6. 复位小车，仍保持支撑且暂停遥控器自动帧。确认启动日志为 `loaded from flash`，
   15 个保存值恢复，速度/转向为零。重新发 `control off` 防止自动启动后继续检查。
7. 改变一个参数但不保存再复位，确认恢复最后一次保存值；改变腿高后 PID 基准保持不变。
8. 在按项目说明准备好的使能控制状态发送 `save`，确认返回 `busy` 且不会中断平衡输出，
   也不会在之后退出控制时自动补写。扶稳并 `control off` 后重新保存。
9. 保存后发送 `control on`，恢复零速度/转向和合适姿态，确认需要重新稳定 500 ms 才使能。
   `control off` 不停止舵机位置保持；它本身也不写入 Flash。

不必为验收故意做 1560 次实板保存或在电机运行时断电。
主机已覆盖日志写满及逐字中断；需要验证真实掉电恢复时应在专门的稳定台架上进行。
显式 `save recycle` 的擦除窗口没有第二个扇区备份，掉电回退编译默认值是已记录的边界。

## 2026-09-13 实板 Flash 验证记录

在电机线与舵机线断开的 STM32F411（512 KB Flash）上完成以下验证。
ST-Link 报告低目标电压，但芯片连接、固件烧录、整段读回和下述保存测试均成功。
测试用 `RelWithDebInfo` 镜像的 BIN 与本次 Release BIN 完全一致，SHA256 为
`339ab862d2137a5b72ae082855847a2b84c7aa87a674646bdd6a5be9d9a71581`。

- 烧录前备份完整 512 KB Flash；烧录后逐字节核对固件，并确认参数扇区未被覆盖。
- 通过 SWD 将文本送入现有 UART 接收队列，由正常命令任务执行 15 个参数的修改和 `save`。
  此次未验证 PC 串口、蓝牙或 nRF 的实际传输链路。
- 读回参数扇区，独立使用 Python `zlib.crc32` 核对记录 CRC、提交标记与全部 15 个字段；
  重复 `save all` 后整个参数扇区保持相同。
- 修改部分 RAM 参数但不保存，复位后全部恢复到上次保存值。
- 用户断开全部供电并重新上电后，重新连接 SWD；确认 Flash 逐字节不变，
  RAM 中全部 15 个参数自动恢复，`Have_saved_parameters=true`。
- 随后恢复测试前默认参数，执行 `save` 并再次复位核对；最终 `unsaved=false`，
  重心基准 9.5°、姿态 Kp 基准 75.35、腿高 44.5 mm、横滚目标 0。
  当前会话最后发送了 `control off`；该开关不持久化，重新上电仍按正常启动条件使能。

本机原始备份、GDB 脚本、Flash/RAM 读回文件和结果 JSON 位于
`build/flash-persistence-20260913/`，其中 `validation-summary.json` 汇总验证结果。
本次没有实车负载测试，也没有在实板上执行日志写满后的 `save recycle` 掉电测试。
