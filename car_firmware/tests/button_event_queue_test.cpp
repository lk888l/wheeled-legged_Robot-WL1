#include <thread>
#include "ButtonEventQueue.hpp"
#include "test_check.hpp"

int main()
{
    app::ButtonEventQueue queue;
    app::ButtonEvent event;
    CHECK(!queue.try_pop(event));
    for (uint32_t i = 0U; i < queue.capacity; ++i) {
        CHECK(queue.push({app::Button::Event::click, i}));
    }
    CHECK(!queue.push({app::Button::Event::long_press, 99U}));
    CHECK(queue.dropped() == 1U);
    for (uint32_t i = 0U; i < queue.capacity; ++i) {
        CHECK(queue.try_pop(event));
        CHECK(event.type == app::Button::Event::click && event.tick == i);
    }
    CHECK(!queue.try_pop(event));

    // Real concurrent producer/consumer exercises publication and slot reuse.
    constexpr uint32_t count = 200000U;
    std::thread producer([&queue] {
        for (uint32_t i = 0U; i < count; ++i) {
            while (!queue.push({app::Button::Event::double_click, i})) {
                std::this_thread::yield();
            }
        }
    });
    for (uint32_t i = 0U; i < count; ++i) {
        while (!queue.try_pop(event)) { std::this_thread::yield(); }
        CHECK(event.type == app::Button::Event::double_click && event.tick == i);
    }
    producer.join();
    CHECK(!queue.try_pop(event));
    std::cout << "queue overflow/FIFO and 200000 concurrent transfers passed; queue="
              << sizeof(queue) << " bytes\n";
}
