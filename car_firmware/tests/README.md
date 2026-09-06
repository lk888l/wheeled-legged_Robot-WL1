# 姿态、启动控制与重心标定验证

Windows PowerShell 可在 `car_firmware` 目录一次运行全部主机回归：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:/rj/CLion 2024.1/bin/mingw/bin/g++.exe'
```

测试分别使用 `-Og` 和 `-O3 -fno-fast-math`，覆盖：

- VQF 首帧到 10 秒静止输入、四种姿态，包含低通滤波初始化阶段；
- 上电稳定 500 ms 才使能、非中位命令/身体运动阻止启动、倾倒和 IMU 异常退出及恢复；
- 根据可调重心基准判断平衡姿态，PID 首帧/重置后的微分及真实 TB6612 驱动的零 PWM 行为；
- 最低腿高基准、左右腿平均值及补偿曲线；
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
print Control_armed
print Control_imu_valid
print Control_pitch_error
print Control_left_pwm
print Control_right_pwm
```

无线命令通过遥控器串口发送。需要固定 `R` 帧目标时，先发送 `joystick off`，
再发送 `R 0 0 0 44.5`；这会明确指定零速度、零转向、零横滚和最低腿高。
`joystick off` 本身只暂停自动发包，小车保留上一组目标。

1. 复位后基准为 `12.6°`；两腿目标初值为 `44.5 mm`。未满足启动条件时两腿
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
