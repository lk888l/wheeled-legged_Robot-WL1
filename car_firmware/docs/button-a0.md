# PA0 按键与业务事件

PA0 板载 KEY 按下接地，配置为上拉输入、低电平有效。`gpio.c`、`main.h`
和 `WL1_F411CEU6.ioc` 同步保存 `KEY_A0` 配置；不启用 PA0 EXTI。

## 默认行为

| 项目 | 默认值 |
| --- | --- |
| 扫描周期 | 5 ms |
| 按下、释放消抖 | 输入连续稳定 20 ms |
| 双击窗口 | 第一次消抖释放到第二次消抖按下，最多 300 ms，含边界 |
| 长按阈值 | 消抖按下后 1000 ms |
| 事件 | `Button::Event::click / double_click / long_press` |
| 事件缓存 | 8 条，每条包含类型和识别时刻的 RTOS tick |
| 按键任务 | `ButtonTask final : public AppTask`，优先级 1，静态栈 128 words |

参数集中在 [TaskConfig.hpp](../Component/UserApp/Tasks/TaskConfig.hpp)。时间参数通过
`pdMS_TO_TICKS` 转换，支持调整 RTOS tick 频率；编译检查禁止零扫描周期及 16-bit tick。
可复用的 [Button](../Component/Application/Button.hpp) 只接收逻辑电平和 `uint32_t`
时间，不依赖 STM32、HAL 或 FreeRTOS。

单击在第一次释放消抖后等待 300 ms；如果此期间完成第二次按下消抖，则在第二次
释放消抖时只产生双击。长按达到阈值时只产生一次，持续按住和随后释放不再产生
click。如果第二次按下变成长按，整组手势只产生 long_press。上电时已经按住的
按键也按正常长按处理；不会自动触发任何电机或舵机动作。

通常每次消抖边沿的延迟为 20–25 ms；单击还必须等待双击窗口，长按从消抖后的
按下开始计时。这些是 CPU 能及时调度时的时序，不能作为过载时的硬实时保证。
状态机使用无符号时间差，正常采样时可跨越 32-bit tick 回绕。

## 对控制任务的影响边界

按键优先级 1，低于 MotionControl 29、CommandService/ServoControl 28，以及
软件定时器任务 2。主控制保持 10 ms、慢环保持 50 ms；原有任务的栈预算不变。

检测路径只有 GPIO 读取、固定状态机和固定队列写入，没有打印、HAL 等待、
动态分配、软件定时器回调、用户回调或向高优先级任务发送通知。状态机和队列的
操作数固定；队列使用单生产者/单消费者的 lock-free 32-bit 原子读写，不屏蔽中断。

高优先级工作延迟扫描时，任务丢弃错过的采样时刻，重新安排下一次休眠，不连续
补扫同一电平。此时按键响应可能变慢、短按可能丢失，控制工作优先。若事件队列满，
丢弃新事件并增加 `drop`，不会等待消费者腾出空间。

按键任务使用 `xTaskCreateStatic`，不消耗 RTOS heap。当前 Arm GNU 15.2.1、
本仓库 FreeRTOS/newlib 配置下，整个 ButtonTask 对象 1200 B，其中栈 512 B；
队列另占 76 B。32 B 的识别状态机已经包含在 ButtonTask 内，不要重复相加。
FreeRTOS TCB 包含 newlib 每任务状态，移植后应重新测量对象大小和栈余量。

## 接入业务

`ButtonEventQueue` 唯一生产者是 ButtonTask，唯一消费者是 CommandServiceTask。
后者在原有 100 ms 周期服务中处理最多 8 条事件；不是按键事件主动唤醒命令任务。
无线 SPI 等待或系统繁忙时，业务收到事件的延迟可超过 100 ms。

在 [CommandServiceTask::handle_button_event](../Component/UserApp/Tasks/CommandServiceTask.cpp)
的三个分支中添加业务请求；默认仅累计计数。消费者上下文的优先级为 28，新增处理
必须有界且不阻塞。耗时操作应交给另一个低优先级 AppTask，执行器请求应通过现有
控制所有者及安全门控，不能在按键分支直接改 GPIO/PWM。不要再添加第二个队列消费者。

若命令通道都不可用，检测仍运行；队列最终填满后按既定策略丢弃新事件。
按键创建失败记入任务失败 bit 4，但不会因此禁用主控制；bit 0–3 的必要任务门控保留。

## 检查方法

串口发送 `button`，例如：

```text
button A0=on click=1 double=1 long=1 drop=0 gap=5 tick
```

`gap` 为启动以来最大实际采样间隔，单位是 RTOS tick，当前 1 tick = 1 ms。
如果超过 20 ms，消抖观察已经不完整，应先检查主任务执行时间、IRQ、外设超时和
输入洪峰。`button` 命令会先读取已有事件，所以可以直接用于人工验收。

主机验证覆盖抖动、单双击边界、长按优先、第三击、计时回绕、队列满载及并发、
RTOS 创建失败，以及扫描任务在被延迟 1000 ms 后仍每次休眠、不连续补扫。
这不等于实机执行时间测试。上板验收应记录控制周期/最坏执行时间、按键任务栈
高水位、满负载下的 `gap/drop`，并对比按键启用前后的控制抖动。
