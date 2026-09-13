#include <array>
#include "TaskReactor.hpp"
#include "test_check.hpp"

struct Sender {
    bool bind_ok{true};
    uint32_t bit{};
    template<typename Signal>
    bool bindReactor(Signal, TaskHandle_t, uint32_t mask) { bit = mask; return bind_ok; }
    void signal(std::function<void()> slot) { slot(); }
};

int main()
{
    fake_rtos::reset();
    TaskReactor reactor;
    Sender rejected{false};
    CHECK(!reactor.connect(&rejected, &Sender::signal, [] {}));
    CHECK(!reactor.connect(static_cast<Sender*>(nullptr), &Sender::signal, [] {}));
    std::array<Sender, 33U> senders{};
    std::array<unsigned, 32U> delivered{};
    for (unsigned i = 0U; i < 32U; ++i) {
        CHECK(reactor.connect(&senders[i], &Sender::signal, [i, &delivered] { ++delivered[i]; }));
        CHECK(senders[i].bit == (uint32_t{1U} << i));
    }
    CHECK(!reactor.connect(&senders[32], &Sender::signal, [] {}));
    fake_rtos::pending_notifications = 0x80000003U;
    try { reactor.taskLoop(); CHECK(false); }
    catch (const fake_rtos::LoopDone&) {}
    CHECK(delivered[0] == 1U && delivered[1] == 1U && delivered[31] == 1U);
    for (unsigned i = 2U; i < 31U; ++i) { CHECK(delivered[i] == 0U); }
}
