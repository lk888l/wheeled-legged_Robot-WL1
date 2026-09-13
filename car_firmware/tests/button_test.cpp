#include <cstdint>
#include <initializer_list>
#include <limits>
#include <vector>

#include "Button.hpp"
#include "test_check.hpp"

using Event = app::Button::Event;

struct Step { bool pressed; uint32_t time; Event event{Event::none}; };

void check_trace(std::initializer_list<Step> trace, uint32_t offset)
{
    app::Button button({20U, 300U, 1000U});
    for (const auto& step : trace) {
        CHECK(button.sample(step.pressed, offset + step.time) == step.event);
    }
}

void scenarios(uint32_t offset)
{
    // Debounce both edges, reject short pulses, then emit one delayed click.
    check_trace({{false,0}, {true,10}, {false,15}, {true,20}, {true,39}, {true,40},
                 {false,100}, {true,105}, {false,110}, {false,129}, {false,130},
                 {false,429}, {false,430,Event::click}, {false,2000}}, offset);
    // A second short press consumes the pending single click.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80},
                 {true,200}, {true,220}, {false,240}, {false,260,Event::double_click},
                 {false,2000}}, offset);
    // The inclusive double-click boundary is based on the debounced press.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80},
                 {true,360}, {true,379}, {true,380}, {false,400},
                 {false,420,Event::double_click}, {false,2000}}, offset);
    // A second press beyond the window produces two independent clicks.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80}, {true,365},
                 {true,380,Event::click}, {true,385}, {false,440}, {false,460},
                 {false,760,Event::click}, {false,2000}}, offset);
    // Late scheduling at the second press still preserves the first click.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80}, {true,365},
                 {true,385,Event::click}, {false,440}, {false,460},
                 {false,760,Event::click}}, offset);
    // Boot-held and sustained buttons emit long press once, never on release.
    check_trace({{true,0}, {true,20}, {true,1019}, {true,1020,Event::long_press},
                 {true,5000}, {false,5005}, {false,5025}, {false,6000}}, offset);
    // A held second press gives the complete gesture long-press precedence.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80}, {true,200}, {true,220},
                 {true,1220,Event::long_press}, {false,1250}, {false,1270}, {false,2000}}, offset);
    // Release debounce does not turn a 995-tick hold into a long press.
    check_trace({{true,0}, {true,20}, {false,1015}, {false,1035},
                 {false,1335,Event::click}}, offset);
    // A long release first observed after a scheduling gap is recognized.
    check_trace({{true,0}, {true,20}, {false,1100},
                 {false,1120,Event::long_press}, {false,2000}}, offset);
    // Triple-click: one double click followed by a single click.
    check_trace({{true,0}, {true,20}, {false,60}, {false,80}, {true,100}, {true,120},
                 {false,160}, {false,180,Event::double_click}, {true,200}, {true,220},
                 {false,260}, {false,280}, {false,580,Event::click}}, offset);
    // Chatter shorter than the debounce interval is not a press.
    check_trace({{true,0}, {false,5}, {true,10}, {false,15}, {false,1000}}, offset);
    // No phantom events following a long press; the next click works normally.
    check_trace({{true,0}, {true,20}, {true,1020,Event::long_press}, {false,1030},
                 {false,1050}, {true,1100}, {true,1120}, {false,1150}, {false,1170},
                 {false,1470,Event::click}}, offset);
}

int main()
{
    // Wrap within debounce, click-window, and long-press measurement.
    for (uint32_t offset : {0U, UINT32_MAX - 10U, UINT32_MAX - 200U, UINT32_MAX - 1000U}) {
        scenarios(offset);
    }
    app::Button button({20U, 300U, 1000U});
    for (unsigned i = 0; i < 1000U; ++i) {
        CHECK(button.sample(true, 100U) == Event::none);
    }
    CHECK(button.sample(false, 105U) == Event::none);
    CHECK(button.sample(false, 2000U) == Event::none);
    std::cout << "48 timing traces and repeated-timestamp debounce passed; recognizer="
              << sizeof(app::Button) << " bytes\n";
}
