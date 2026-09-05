#include "BoardHardware.hpp"
#include "Tasks/ButtonTask.hpp"
#include "test_check.hpp"

namespace {
bsp::BoardHardware board;
unsigned delay_calls{};
void advance()
{
    ++delay_calls;
    if (delay_calls == 5U) { fake_rtos::now += 1000U; } // Higher-priority work wins.
    board.pressed = fake_rtos::now < 1200U;
    if (delay_calls == 100U) { throw fake_rtos::LoopDone{}; }
}
}

int main()
{
    fake_rtos::reset();
    app::ButtonEventQueue events;
    app::ButtonTask button(board, events);
    board.pressed = true;
    fake_rtos::run_on_resume = true;
    fake_rtos::on_delay = advance;
    try { (void)button.start(); CHECK(false); }
    catch (const fake_rtos::LoopDone&) {}
    CHECK(fake_rtos::static_creates == 1U && fake_rtos::dynamic_creates == 0U);
    CHECK(fake_rtos::notifications == 0U && fake_rtos::critical_entries == 0U);
    CHECK(fake_rtos::nonblocking_delays == 0U); // No catch-up spin after starvation.
    CHECK(button.max_sample_gap_ticks() == 1005U);
    app::ButtonEvent event;
    CHECK(events.try_pop(event) && event.type == app::Button::Event::long_press);
    CHECK(!events.try_pop(event));
}
