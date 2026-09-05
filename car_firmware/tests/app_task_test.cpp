#include <type_traits>
#include "AppTask.hpp"
#include "test_check.hpp"

class TestTask final : public app::AppTask {
public:
    explicit TestTask(app::AppTaskConfig config, StackType_t* stack = nullptr, StaticTask_t* tcb = nullptr)
        : AppTask(config, stack, tcb) {}
    unsigned runs{};
private:
    void run() override
    {
        CHECK(is_running());
        CHECK(fake_rtos::scheduler_depth == 0U);
        ++runs;
        throw fake_rtos::LoopDone{};
    }
};

int main()
{
    static_assert(std::is_abstract_v<app::AppTask>);
    static_assert(!std::is_copy_constructible_v<TestTask> && !std::is_move_constructible_v<TestTask>);
    const app::AppTaskConfig config{app::TaskId::heartbeat, "Test", 128U, 1U};
    fake_rtos::reset();
    TestTask dynamic(config);
    CHECK(!dynamic.notify_give() && !dynamic.notify_value(7U));
    fake_rtos::fail_create = true;
    CHECK(!dynamic.start());
    CHECK(!dynamic.is_running() && fake_rtos::scheduler_depth == 0U);
    fake_rtos::fail_create = false;
    CHECK(dynamic.start());
    CHECK(!dynamic.start());
    CHECK(fake_rtos::dynamic_creates == 2U);
    CHECK(dynamic.notify_value(0x12345678U, eSetBits));
    CHECK(fake_rtos::notified_handle == dynamic.handle());
    CHECK(fake_rtos::notified_value == 0x12345678U && fake_rtos::notified_action == eSetBits);
    CHECK(dynamic.notify_give() && fake_rtos::notified_action == eIncrement);

    fake_rtos::reset();
    StackType_t stack[128]{};
    StaticTask_t tcb;
    TestTask statically_allocated(config, stack, &tcb);
    fake_rtos::run_on_resume = true;
    try { (void)statically_allocated.start(); CHECK(false); }
    catch (const fake_rtos::LoopDone&) {}
    CHECK(statically_allocated.runs == 1U && statically_allocated.is_running());
    CHECK(fake_rtos::static_creates == 1U && fake_rtos::dynamic_creates == 0U);
    CHECK(fake_rtos::supplied_stack == stack && fake_rtos::supplied_tcb == &tcb);

    TestTask bad_name({app::TaskId::button, nullptr, 128U, 1U});
    TestTask empty_name({app::TaskId::button, "", 128U, 1U});
    TestTask bad_stack({app::TaskId::button, "Bad", 0U, 1U});
    TestTask bad_priority({app::TaskId::button, "Bad", 128U, configMAX_PRIORITIES});
    TestTask missing_tcb(config, stack);
    CHECK(!bad_name.start() && !empty_name.start() && !bad_stack.start());
    CHECK(!bad_priority.start() && !missing_tcb.start());
}
