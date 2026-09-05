# 工程审查与本次改造记录 — 2026-09-05

本次实现范围是车体端 `car_firmware`：PA0 按键、AppTask 业务任务封装及与拆分
直接相关的问题修复。审查覆盖两端自有应用、BSP/驱动、FreeRTOS 配置、构建、
测试与文档；对 HAL、FreeRTOS、ETL 和 VQF 检查其接入边界，没有逐行认证第三方库。
遥控器端只读审查并完成构建。本次没有烧录，也没有把历史上板记录当作本次验证。

## 结论

车体端任务已经从约 800 行的 `main.cpp` 拆为五个继承 `AppTask` 的 `final` 任务类。
入口只负责装配、初始化、必要任务门控和启动。按键扫描低于控制及定时器优先级，
使用静态存储和有界事件队列；输入过载时让按键延迟或丢事件，不增加控制任务工作。

软件构建和主机测试可以确认接口、状态机、资源使用和调度调用方式；不能据此承诺
实机最坏执行时间或“零影响”。下表 C01–C04 仍是运行期可靠性的优先整改项。

## 本次已修复

| 问题 | 修改与证据 |
| --- | --- |
| 车体 PA0 未配置、未检测 | 增加上拉输入、BSP 逻辑电平读取、按键状态机和静态任务；`.ioc` 同步 |
| AppTask 只包装自由函数，业务集中于入口 | 抽象 `run()`、禁止拷贝/移动、显式依赖注入，五个任务各自实现业务；创建期间先发布句柄再恢复调度 |
| 控制全循环屏蔽 IRQ | 共享参数、姿态和腿目标通过短临界区复制；I2C、VQF、PID 和格式化都在临界区外。旧代码连 TIM5 HAL tick 也被屏蔽，超时路径可能永不结束 |
| PID 首次微分读取未初始化内存 | `prev_actual` 显式初始化为 0；使用四种预填内存模式验证首次输出和 reset 后输出一致 |
| TaskReactor 越界访问 | 原 ETL vector 逻辑长度缩到 1 后仍访问 32 槽；改为真正的固定 32 槽数组，测试第二槽、第 32 槽、溢出和绑定失败 |
| 多字段遥控目标更新不一致 | `R` 完整解析成功后一次发布参数副本；MotionControl 读取一致快照 |
| 短参数移除前缀越界、数字后缀被接受 | 统一完整 token 解析，拒绝空值、尾随字符、溢出、NaN/Inf，失败保留原参数；优化构建也验证有限数检查 |
| 命令表重复键/横滚比例项写错 | 删除重复 `target_roll`，`rollpid -p` 正确更新 Kp；固定分支分发不再构建动态处理器哈希表 |
| nRF 遥测输出范围错误 | 旧 `to_chars` 终点在缓冲区起点之前，终止符也有跳位；改为有界、补零的四字段编码，覆盖溢出和邻接内存保护测试。使用整数格式化控制 Flash 增量 |
| motor 命令应答与实际行为不符 | 当前 PID 从未消费原始 PWM 通知；明确拒绝该命令，不绕过闭环及安全门控 |
| 启动尾部可能覆盖运行故障 | `runtime_fault` 锁存，后续 ready/enable 请求不能重新放行 |
| 头文件重复包含失效 | 修正 LegKinematics 不匹配的 include guard，加入重复包含回归验证 |

## 尚未修复的审查发现

P1：应在实际运行验收前解决；P2：确定的维护/稳定性风险，需要专门验证。这里的
优先级是工程判断，不是功能安全等级。未自动调整控制标定、失联动作或底层 DMA
恢复策略，因为这些改变需要明确业务策略和硬件故障测试，不属于按键所需的集成。

| ID / 优先级 | 证据与触发条件 | 影响与建议 |
| --- | --- | --- |
| C01 / P1 | [MPU6050.h](../Component/HardWare/IMU/MPU6050.h) `MPU6050_TIME_OUT=500`；[MotionControlTask](../Component/UserApp/Tasks/MotionControlTask.cpp) 每 10 ms 同步读取 IMU | I2C 故障仍可让最高优先级控制任务忙等数百 ms；停止轮输出是在读取返回后。大临界区死锁已修复，但期限违约仍存在。区分初始化/运行超时，限定整次采样期限，验证断线、SDA 拉低和失败停机时限 |
| C02 / P1 | [CommandServiceTask](../Component/UserApp/Tasks/CommandServiceTask.cpp) 持续保留最后目标；两端协议没有命令序号、有效期或车端命令年龄门控 | 遥控器断电/失联后可继续保持旧速度与转向。应约定失联时间、减速和继续平衡策略，再做掉包/重连测试，不能简单把失联等同于关闭平衡力矩 |
| C03 / P1 | 两端 [车端 nRF 驱动](../Component/HardWare/Telecontrol/NRF24L01P.cpp) / [遥控器 nRF 驱动](../../tele_firmware/Component/HardWare/Telecontrol/NRF24L01P.cpp) 的 SPI DMA 等待超时直接返回，寄存器地址等使用栈缓冲 | 超时并不证明 DMA 已停止，栈对象生命周期结束后 DMA 仍可能访问它；迟到的完成信号还可能污染下一事务。需要中止并确认完成、清理信号量和 HAL 状态，或使用驱动拥有的持久事务缓冲，覆盖错误 IRQ/丢完成 IRQ 测试 |
| C04 / P1 | 参数解析已拒绝非有限数，但 [CommandServiceTask](../Component/UserApp/Tasks/CommandServiceTask.cpp) 对有限的增益/目标没有物理范围限制；[PID](../Component/UserApp/CtrlAlgorithm/PID.hpp) 未处理运算溢出 | 例如极大有限 Kp/Kd 与误差相乘仍可生成 Inf/NaN，后续浮点转 PWM 整数可能越界。建立每参数范围及运行输出有限数检查；先确定机械、增益和目标的允许范围 |
| C05 / P2 | [LkUart.hpp](../Component/Peripheral/LkUart.hpp) 在 DMA 启动前设 `dmaBusy_`，忽略启动/重启返回值；RX 回调未区分 HT 与 IDLE/TC | HAL_BUSY/ERROR 后 TX 可能永久停滞，RX 半传输时可能提前交付仍在写入的缓冲。需统一缓冲所有权、过滤 HT/管理帧边界并验证 DMA 错误恢复；当前短命令正常路径不代表故障路径安全 |
| C06 / P2 | [MotionControlTask](../Component/UserApp/Tasks/MotionControlTask.cpp) 校准腿高仍为 `(left + left) / 2`，自动生成 angle Kp/bias | 横滚时只使用左腿影响标定；`anglepid -p`/`anglebias` 存储值不参与当前自动校准。保留原标定行为，建议分别提供自动校准/人工调参模式，并在改为真实平均值后重新验证控制响应 |
| C07 / P2 | [Servo.cpp](../Component/HardWare/Motor/Servo.cpp) 的高优先级任务与低优先级软件定时器共享 TargetAngle/StepSize/CurrentAngle，定时器命令返回值未检查 | 可能观察混合目标或丢失定时器启动/停止请求。建议由一个任务拥有舵机状态，或建立小范围一致快照与错误反馈；特别验证控制故障发生在平滑回调期间 |
| C08 / P2 | [FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h) 未开启栈溢出/分配失败钩子，板级未启用独立看门狗；[CMakeLists](../CMakeLists.txt) Release 仍使用 `-Ofast` | 缺少故障定位和复位兜底；fast-math 不保留全部 IEEE 浮点语义。按测得的 WCET、堆余量和栈高水位确定监测预算，再比较 `-O3`/`-Ofast`，不要只凭镜像大小调整控制配置 |
| C09 / P2 | 两端 nRF/UART 的实现已经分叉，遥控器任务仍为 [main.cpp](../../tele_firmware/Component/UserApp/main.cpp) 自由函数；仓库没有 CI 工作流 | 本次新任务抽象只改车端。后续优先抽取协议编解码和纯按键逻辑等稳定模块，建立两端构建与主机回归 CI，再迁移遥控器任务；不应直接用车端旧驱动覆盖遥控器已有恢复逻辑 |

## 验证记录

使用 Arm GNU Toolchain 15.2.1、主机 GCC 16.1.0、CMake/Ninja。
主机测试包括初始化管理、命令解析、48 条按键时序（含回绕）、重复时间戳消抖、
20 万次 SPSC 并发传输、队列溢出、遥测边界、AppTask 生命周期、事件槽容量、
共享状态/故障锁存、真实 ButtonTask 循环的 RTOS 替身测试，以及 PID 首次采样。
另外直接执行 CommandServiceTask 和 MotionControlTask 的真实任务体（替换 RTOS/HAL），
覆盖命令更新与回滚、遥测和按钮消费、10/50 ms 调度调用、IMU 连续三次失败停机，
以及 HAL 读取不在临界区中。

构建与最终资源结果如下。链接 RAM 包含保留的 32 KiB RTOS heap，
不代表实机堆峰值、CPU 使用率或剩余任务栈空间。

| 构建 | 结果 |
| --- | --- |
| 原始车端 Debug | 通过；Flash 245356 B，链接 RAM 46864 B |
| 改造车端 Debug | 通过；Flash 238624 B，链接 RAM 48520 B；相比基线 Flash −6732 B，RAM +1656 B |
| 改造车端 Release | 通过；Flash 233332 B，链接 RAM 48528 B |
| 主机 Debug / `-Ofast` | 各 12 组回归测试通过；断言在优化构建中保持启用 |
| 遥控器 Debug | 通过；Flash 147356 B，链接 RAM 41576 B；未修改遥控器源码 |

待上板验证：PA0 空闲/按下电平及真实接点抖动；控制周期、WCET 和 IRQ 延迟；
ButtonTask 的栈高水位及 `button` 输出的最大采样间隔；UART/nRF 满负载；
IMU 与无线断线；控制故障期间两路轮 PWM、方向脚和舵机通道的最终状态。

## 采用的规范依据

采用显式接口与资源所有权、禁止复制任务对象、短临界区、有界存储和可测试的硬件
边界，参考当前 [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
的 C.35、C.67、CP.43、CP.200。这里是针对单核 FreeRTOS 的渐进改造，不声明
MISRA、AUTOSAR 或功能安全认证符合性。

静态栈/TCB 使用方式核对了本地 FreeRTOS 10.3.1 实现及
[FreeRTOS 任务创建文档](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate)；
缩小临界区参考 [FreeRTOS Kernel Book 第 8 章](https://github.com/FreeRTOS/FreeRTOS-Kernel-Book/blob/main/ch08.md)。
