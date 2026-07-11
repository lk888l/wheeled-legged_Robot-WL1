/********************************************************************************
  * @file           : LKButton.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-4-21
  *******************************************************************************/


#ifndef __TELE_LKBUTTON_HPP
#define __TELE_LKBUTTON_HPP

#include "BasicObject.hpp"
#include "etl/vector.h"
#include "main.h"

class LKButton : BasicObject{
    static constexpr uint8_t DMA_BUF_SIZE = 20;
    static constexpr uint16_t LONG_PRESS_TIME = 1000;
    static constexpr uint16_t DOUBLE_CLICK_TIME = 250; // 双击间隔阈值 250ms
    static constexpr uint16_t active_pins_mask = 0xFFFF; // 哪些位需要检测
public:
    uint16_t key_dma_buf[DMA_BUF_SIZE]; // 存放 GPIOx->IDR 的数据
public:
    enum class ButtonEvent {
        SingleClick,
        DoubleClick,
        LongPress,
        Release
    };
// 状态机定义
    enum class KeyState {
        Idle,
        Pressed,
        WaitDouble,
        LongPressed
    };

    LKButton(TIM_HandleTypeDef *htim,DMA_HandleTypeDef * hdma, GPIO_TypeDef *btnP)
        : H_TIM(htim),H_DMA(hdma),BtnPort(btnP)
    {
        HAL_DMA_Start_IT(H_DMA, (uint32_t)BtnPort->IDR, (uint32_t)key_dma_buf, DMA_BUF_SIZE);
        // 开启定时器的 DMA 请求（注意是请求，不是中断）
        __HAL_TIM_ENABLE_DMA(H_TIM, TIM_DMA_UPDATE);
        // 启动定时器
        HAL_TIM_Base_Start(H_TIM);
    }

    /**
     * @brief
     * @param pinnum
     * @return
     */
    bool Push_Pin_vec(uint8_t pinnum){
        if(Pin_vec.full())  {return false;}
        Pin_vec.push_back(pinnum);
        return true;
    }

    /**
     * @brief 处理逻辑（由 FreeRTOS 任务调用）
     * @param notified_value
     * @param dma_buf
     * @param buf_size
     */
    void process(uint16_t* dma_buf, uint8_t buf_size) {
        uint32_t ulNotifiedValue;
        uint16_t stable_state;
        uint16_t last_stable_state = 0xFFFF; // 假设默认高电平

        uint8_t start_idx = (ulNotifiedValue == 0) ? 0 : (DMA_BUF_SIZE / 2);
        uint8_t end_idx = start_idx + (DMA_BUF_SIZE / 2);

        // 1. 并行消抖：只有当连续10个采样点全为0，才判定为按下；全为1判定为释放
        uint16_t mask_and = 0xFFFF; // 找稳定的1
        uint16_t mask_or  = 0x0000; // 找稳定的0

        for (int i = start_idx; i < end_idx; i++) {
            mask_and &= key_dma_buf[i];
            mask_or  |= key_dma_buf[i];
        }

        // 更新稳定状态 (如果 mask_or == 0 说明稳定为低电平，如果 mask_and == 1 说明稳定为高电平)
        // 巧妙的位运算：保留上次状态，除非满足全0或全1才更新
        stable_state = (last_stable_state & mask_or) | mask_and;

        uint16_t changed_bits = stable_state ^ last_stable_state;
        last_stable_state = stable_state;

        uint32_t current_tick = xTaskGetTickCount();

        // 2. 状态机分发 (高并发处理，遍历16个可能发生变化的位)
        for(const auto& i : Pin_vec){
            uint8_t is_pressed = !(stable_state & (1 << i)); // 假设低电平按下
            uint8_t is_changed = (changed_bits & (1 << i));

            KeyCB_t *k = &keys[i];

            switch (k->state) {
                case KeyState::Idle:
                    if (is_changed && is_pressed) {
                        k->state = KeyState::LongPressed;
                        k->press_time = current_tick;
                    }
                    break;

                case KeyState::Pressed:
                    if (is_changed && !is_pressed) { // 释放
                        if (current_tick - k->press_time < LONG_PRESS_TIME) {
                            k->state = KeyState::WaitDouble;
                            k->release_time = current_tick;
                        } else {
                            // 已经是长按释放，重置状态
                            k->state = KeyState::Idle;
                        }
                    } else if (is_pressed && (current_tick - k->press_time >= LONG_PRESS_TIME)) {
                        // 触发长按
                        // TODO: 触发长按事件 (EventGroup / Queue / Callback)
//                        printf("Key %d Long Pressed!\n", i);
                        k->state = KeyState::LongPressed;
                    }
                    break;

                case KeyState::WaitDouble:
                    if (is_changed && is_pressed) { // 在等待时间内再次按下
                        // 触发双击
                        // TODO: 触发双击事件
//                        printf("Key %d Double Clicked!\n", i);
                        k->state = KeyState::Pressed; // 重新进入按下状态，防止连按逻辑死锁
                    } else if (current_tick - k->release_time >= DOUBLE_CLICK_TIME) {
                        // 等待超时，确认为单击
                        // TODO: 触发单击事件
//                        printf("Key %d Single Clicked!\n", i);
                        k->state = KeyState::Idle;
                    }
                    break;

                case KeyState::LongPressed:
                    if (is_changed && !is_pressed) { // 长按后释放
                        k->state = KeyState::Idle;
                    }
                    break;
            }
        }
    }

    template<typename SignalPtr>
    bool bindReactor(SignalPtr signalFunc, TaskHandle_t task, uint32_t bitMask) {
        if constexpr (std::is_same_v<SignalPtr, decltype(&LKButton::signal_Init)>) {
            if (signalFunc == &LKButton::signal_Init) {
                Init_cfg.task_h = task;
                Init_cfg.bitMask = bitMask;
                return true;
            }
        }
        return false;
    }

    void signal_Init(){

    }

private:
    struct KeyCB_t {
        KeyState state;
        uint32_t press_time;
        uint32_t release_time;
    };
    DMA_HandleTypeDef *H_DMA = nullptr;
    TIM_HandleTypeDef *H_TIM = nullptr;
    GPIO_TypeDef *BtnPort = nullptr;
    KeyCB_t keys[16];
    etl::vector<uint8_t,16> Pin_vec;
    uint16_t last_stable = 0xFFFF;
    /* signal config define */
    SignalContext Init_cfg{};
    SignalContext Click_cfg{};
    SignalContext DoubleClick_cfg{};
    SignalContext LongPressed_cfg{};
};


#endif //__TELE_LKBUTTON_HPP
