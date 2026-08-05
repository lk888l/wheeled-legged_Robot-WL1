# 固件开发规范

本文定义 `car_firmware` 的代码所有权、依赖方向、实时性约束和验证门槛。
目标是在不改变无线协议、控制公式和 CubeMX 外设配置的前提下，让后续修改可定位、
可测试、可由 CubeMX 重新生成。

## 分层与依赖方向

```text
UserApp (composition)
    ↓
App + AppModules
    ↓
Control / HardWare / Peripheral
    ↓
Base / ETL / STM32 HAL / FreeRTOS
```

- `Component/UserApp/main.cpp` 只注册模块，不放设备驱动、命令表或控制循环；
- `Component/App` 不依赖 HAL/FreeRTOS，因而可以直接进行主机单元测试；
- `Component/AppModules` 隐藏具体任务类和共享运行状态，只公开模块工厂；
- `HardWare` 封装设备，`Peripheral` 封装 MCU 通信外设，`CtrlAlgorithm` 保持纯算法；
- 下层组件不能反向包含 `AppModules` 或 `UserApp`。

`AppManager` 使用固定 8 槽指针表，不分配 C++ 堆内存。当前三个 legacy 任务只能在
调度器启动前安全回滚；调度器运行后不支持强制卸载模块。若以后需要运行时停止，
必须先让任务协作退出，并清理 PWM、软件定时器、DMA 和信号量。

## CubeMX 所有权

- `Core/`、`Drivers/`、`Middlewares/` 和 `.ioc` 由 CubeMX 管理；
- 对 `Core/` 的手写修改只能位于 `USER CODE` 区域；
- `CMakeLists_template.txt` 只接收 CubeMX 生成的 source/include/define 列表，
  稳定的编译、链接和组件策略放在 `cmake/firmware.cmake`；
- 手写组件源码不使用递归 glob，必须在对应组件的 `CMakeLists.txt` 显式登记；
- 重新生成后按[故障排查文档](troubleshooting.md#cubemx-重新生成检查项)复核。

## C/C++ 规范

- C11、C++23，4 空格缩进、Allman 大括号、100 列上限；格式见 `.clang-format`；
- 新头文件使用 `#pragma once` 或非保留形式的 include guard，不使用 `__NAME`；
- C 与 C++ 的边界保持最小化；当前只公开 `CPP_Main(void)`；
- 组件只公开稳定接口，具体任务类和单例放在 `src/`；
- 控制周期内不使用 `new`、`malloc`、无界容器或阻塞式日志；
- 第三方 ETL 作为系统头处理，项目手写代码启用 `-Wall -Wextra -Wpedantic`。

## FreeRTOS 与中断

- 禁止在临界区内调用 `vTaskDelay*()`、队列阻塞、慢速外设或格式化；
- ISR 只调用 `...FromISR` API，并正确传递 `higher_priority_task_woken`；
- 调用 FreeRTOS ISR API 的 NVIC 数值优先级必须不小于 5；
- `volatile` 不提供互斥或多字段一致性；需要事务语义时复制短快照或显式同步；
- 任务栈参数单位是 32-bit word，不是 byte；新增任务要记录优先级、栈和唤醒源；
- 电机/舵机任务失败时应优先进入安全输出，不能继续使用无效传感器数据。

## 新增应用模块

1. 在 `Component/AppModules/include/app_modules.hpp` 声明模块工厂；
2. 在 `Component/AppModules/src` 定义具体模块类，保持实现私有；
3. 在 `Component/AppModules/CMakeLists.txt` 显式加入源文件和依赖；
4. 在 `Component/UserApp/main.cpp` 按依赖顺序注册；
5. 为纯状态机或算法补充 `tests/` 主机测试；
6. 完成 Debug、Release、主机测试和系统板验证；
7. 涉及控制方向、机构或协议时，再完成架空整车测试。

## 验证命令

```sh
cmake -S . -B build/Debug -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --parallel

cmake -S . -B build/Release -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --parallel

cmake -S tests -B build/host-tests -G "MinGW Makefiles"
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure

git diff --check
```

安装旧版 ST-LINK Utility 且 CMake 能找到 `ST-LINK_CLI` 时，可以执行：

```sh
cmake --build build/Release --target flash
```

烧录只证明镜像可写入且校验通过。仅连接系统板时，可以验证 MCU、任务创建和内核
存活；IMU、nRF、编码器、电机和舵机功能必须在相应硬件接入后单独验证。
