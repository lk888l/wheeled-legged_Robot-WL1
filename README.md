# WL1 轮腿机器人

[中文](README.md) | [English](README_en.md)

固件文档：[小车端固件](car_firmware/README.md) | [遥控器固件](tele_firmware/README.md)

WL1 是一套基于 STM32F411CEU6 的低成本轮腿机器人项目。仓库包含小车端与
手持遥控器端固件、控制算法与腿部运动学仿真，以及用于在线调参的 VOFA+
界面配置。

## 项目结构

```text
wheeled-legged_Robot-WL1/
├── car_firmware/          # 小车端固件
├── tele_firmware/         # 手持遥控器端固件
├── simulation/            # LQR 与腿部运动学仿真
├── vofa_host_tools_cfg/   # VOFA+ 命令组和调参窗口配置
├── LICENSE
├── README.md
└── README_en.md
```

| 目录 | 主要内容 | 入口 |
| --- | --- | --- |
| `car_firmware/` | MPU6050、编码器、串级 PID、电机、舵机、nRF24L01+ 接收和串口调参 | [小车固件 README](car_firmware/README.md) |
| `tele_firmware/` | 摇杆、按键、OLED、nRF24L01+ 发送和遥控指令编码 | [遥控器固件 README](tele_firmware/README.md) |
| `simulation/` | Python/MATLAB LQR 参数计算，以及四连杆腿部运动学可视化 | 本文“仿真工具”一节 |
| `vofa_host_tools_cfg/` | VOFA+ 命令组 `vofa.cmds.json` 和窗口布局 `vofa_tab.json` | 本文“VOFA+ 调参配置”一节 |

两个固件工程采用相似的目录约定：

| 路径 | 说明 |
| --- | --- |
| `Component/` | 项目维护的硬件驱动、外设封装、控制算法和应用代码 |
| `Core/` | STM32CubeMX 生成的启动、外设初始化和中断代码 |
| `Drivers/` | STM32 HAL 与 CMSIS |
| `Middlewares/` | FreeRTOS 等中间件 |
| `*.ioc` | STM32CubeMX 工程配置 |
| `CMakeLists.txt`、`cmake/` | Arm GNU Toolchain 命令行构建配置 |

固件内部的任务划分、硬件引脚、烧录方法和调试步骤，请直接查看上表中的两个
固件 README。

## 命令行编译

仓库根目录不是一个聚合 CMake 工程，两个固件需要分别配置和编译。

### 依赖

- CMake 3.28 或更新版本；
- Ninja；
- Arm GNU Toolchain，工具前缀为 `arm-none-eabi-`；
- OpenOCD 和 ST-Link（仅烧录或调试时需要）；
- STM32CubeMX（仅修改 `.ioc` 或重新生成外设代码时需要）。

确认主要构建工具已加入 `PATH`：

```sh
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

### Release 构建

在仓库根目录依次执行：

```sh
# 小车固件
cmake -S car_firmware -B car_firmware/build/Release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build car_firmware/build/Release --parallel

# 遥控器固件
cmake --preset Release -S tele_firmware
cmake --build tele_firmware/build/Release --parallel
```

如需调试版本，将命令中的 `Release` 全部替换为 `Debug`。切换 CMake 生成器时，
应使用新的构建目录，避免复用已有的 CMake 缓存。

主要输出如下：

| 固件 | 输出目录 | ELF 文件 |
| --- | --- | --- |
| 小车端 | `car_firmware/build/<配置>/` | `WL1_F411CEU6.elf` |
| 遥控器端 | `tele_firmware/build/<配置>/` | `WL1_F411CEU6_Tele.elf` |

同一目录还会生成用于烧录或发布的 `.hex`、`.bin` 文件，以及用于内存分析的
`.map` 文件。OpenOCD 烧录命令见[小车固件 README](car_firmware/README.md#3-使用-st-link-烧录)
和[遥控器固件 README](tele_firmware/README.md#2-使用-st-link-烧录)。

## 仿真工具

`simulation/` 当前包含：

- `Leg_kinematics.py`：交互式显示四连杆腿部机构和轮心轨迹；
- `WL1_LQR.py`：使用 Python Control Systems Library 计算 LQR 增益；
- `WL1_LQR.m`：使用 MATLAB `lqr` 计算 LQR 增益。

Python 脚本依赖 `numpy`、`matplotlib` 和 `control`：

```sh
python -m pip install numpy matplotlib control
python simulation/Leg_kinematics.py
python simulation/WL1_LQR.py
```

MATLAB 脚本需要提供 `lqr` 函数的 Control System Toolbox。修改车体质量、
重心高度、轮径或腿部几何尺寸后，应重新计算或验证仿真参数，并同步检查小车
固件中的对应常量。

## VOFA+ 调参配置

`vofa_host_tools_cfg/` 中的两个 JSON 文件需要在 VOFA+ 中分别导入：

- `vofa.cmds.json`：PID、姿态偏置、腿高等参数的命令组；
- `vofa_tab.json`：滑块、波形、姿态方块和摇杆等窗口布局。

小车端 USART1 默认使用 `115200, 8-N-1`。连接串口后即可通过配置好的控件
发送调参命令；具体命令、范围和建议调参顺序见
[小车固件命令参考](car_firmware/docs/commands.md)。

> [!WARNING]
> 自平衡任务启动后可能直接输出电机 PWM。首次烧录、检查反馈方向或在线调整
> PID 时，请架空车轮、断开电机功率或使用限流电源，并确保可以立即断电。

## 协同修改约定

- 两端使用相同的 32 字节 nRF24L01+ 协议；修改地址、频道、速率、payload
  长度或 `R` 命令字段时，必须同步更新两个固件；
- 修改 CubeMX 配置后，应检查 `USER CODE` 区域和 CMake 源文件列表是否仍然完整；
- 腿部尺寸、控制符号或机械方向变更时，应同时核对仿真、遥控显示和小车控制；
- 提交前建议至少完成两端 Debug/Release 构建，并确认未提交本地构建目录。

## 许可证

本项目采用 [MIT License](LICENSE)。
