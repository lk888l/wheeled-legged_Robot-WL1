# 舵机安装与 IMU 验证（2026-09-16）

## 变更

- 新增 `install on|off|status`。安装中独立保持 44.5 mm，双轮输出为零，禁止遥控与自适应腿高覆盖。
- 进入安装与 PWM 最终写入互锁；离开安装保持控制关闭，清空旧目标，恢复控制重新初始化融合与稳定门控。
- 修复舵机 PWM 停止后无法重新启动、残留定时器回调可能重写 compare 的问题。
- MPU6050：复位、陀螺仪 X PLL 时钟、100 Hz/DLPF=3、寄存器回读、按量程换算、14 字节一致采样、5 ms 有界 I²C 超时。
- 删除固定零偏 `(2.5, 0.7, 0.9)`。实板测到静态 Z 角速度约 3.4 °/s，超过原 VQF `biasClip=2`，原静止检测一直为 false。调整 `biasClip=5`、`biasSigmaInit=2`，保留原静止方差/时间条件，复验静止检测为 true，零偏估计收敛到传感器读数。

寄存器依据 [TDK MPU-6000/6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)。
VQF 参数依据 [官方参数说明](https://vqf.readthedocs.io/en/stable/ref_cpp_params.html)：biasClip 同时限制估计范围与静止检测，不能简单删除固定补偿而仍保留过小的估计范围。

## 自动验证

- Debug、Release 各 50 项 CTest 通过，包括真实命令/运动/舵机任务组合、大零偏融合收敛、驱动寄存器与 I/O 失败，以及舵机停止/重新启动。
- Debug、Release、RelWithDebInfo 固件构建通过；后两者 BIN 完全相同。
- 小程序 137 项 Node 测试通过，TypeScript、微信开发者工具 WXML/WXSS 编译通过；覆盖分片回执、超时、旧连接回执、状态锁定与模拟模式。

## 实板验证

用户确认电机线已断开。ST-Link V2J35M26 报告低 Vref，HLA 后端连接、擦写、读回均成功。
烧录前备份完整 512 KB Flash；最终 BIN 272232 字节，SHA256：

```text
d15ad88da13764e4c5aa858566ebed9c16de8d07e91ddbd4a7e62eeeb958ce5d
```

最终 Flash 前 272232 字节与 BIN 逐字节相同，扇区 7（128 KB 参数区）与原备份完全相同，没有保存或改写原 PID 参数。

通过 SWD 向现有 UART 接收队列注入命令，由真实 CommandService 解析执行，未跳过命令业务：

- `install on` 后 active=1、ready=1、control=off；TIM1 CCR1/CCR2 始终为 0。
- TIM9 CCR1/CCR2 固定为 513 / 2365；CC1E、CC2E 使能，左右逻辑角为约 1.188965°。
- 安装中 `R 90 90 18 78.5`、`legheight 78.5`、`control on` 均不能改变上述状态，`control off` 继续保持舵机。
- 无命令等待 2 秒仍锁定；`install off` 后轮电机/舵机 compare 全部为 0，控制未自动开启。
- 再次 `install on` 成功恢复两路舵机 PWM。最终设备保持在安装模式。
- 正常采样时 IMU 有效，10 ms 控制周期，观测窗口内最大采样间隔 10 tick、deadline_misses=0。
- 静置收敛后，间隔约 2 秒读取 5 组姿态，当前约 -83.5° 俯仰放置条件下 Roll/Pitch/Yaw 峰峰值分别约 0.193° / 0.013° / 0.197°，5 次静止检测均为 true。这是静态短窗口观测，不代表动态负载精度。

原始备份、烧录日志、GDB 脚本、读回文件、静止观测位于 `car_firmware/build/install-*`。

## 验证边界

此处验证了主机逻辑、实板命令执行、Flash 保留及电机/舵机寄存器。
没有实际电机负载、舵机角度传感器或手机 BLE 真机联调；界面“已锁定”表示 PWM 软件目标到位，需要目视确认舵机静止。
小程序修改位于独立仓库 `D:/kk/wechat-app/cf-studio_wechat-app`。
安装开关不持久化，设备重新上电需重新进入安装模式。
