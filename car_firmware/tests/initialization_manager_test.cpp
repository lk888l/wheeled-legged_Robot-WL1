#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>

#include "InitializationManager.hpp"

namespace {

struct FakeModule {
    uint8_t call_order{};
    bool succeeds{};
    uint8_t* next_call_order{};
};

struct TestContext {
    uint8_t observer_calls{};
};

bool initialize_fake(void* context)
{
    auto* module = static_cast<FakeModule*>(context);
    assert(module != nullptr);
    assert(module->next_call_order != nullptr);
    module->call_order = (*module->next_call_order)++;
    return module->succeeds;
}

void observe(const app::InitializationStep&, bool, void* context)
{
    auto* test = static_cast<TestContext*>(context);
    assert(test != nullptr);
    ++test->observer_calls;
}

void test_failure_does_not_short_circuit()
{
    uint8_t next_call_order = 1U;
    FakeModule first{0U, true, &next_call_order};
    FakeModule failed{0U, false, &next_call_order};
    FakeModule last{0U, true, &next_call_order};
    const std::array steps{
        app::InitializationStep{0U, "first", initialize_fake, &first},
        app::InitializationStep{1U, "failed", initialize_fake, &failed},
        app::InitializationStep{2U, "last", initialize_fake, &last},
    };
    TestContext context{};

    const app::InitializationReport report = app::InitializationManager{}.initialize_all(
        steps.data(), steps.size(), observe, &context);

    assert(first.call_order != 0U);
    assert(failed.call_order == first.call_order + 1U);
    assert(last.call_order == failed.call_order + 1U);
    assert(context.observer_calls == steps.size());
    assert(report.attempted_mask == 0x07U);
    assert(report.failed_mask == 0x02U);
    assert(report.succeeded(0U));
    assert(!report.succeeded(1U));
    assert(report.succeeded(2U));
    assert(!report.all_succeeded());
}

void test_duplicate_id_is_configuration_failure_but_iteration_continues()
{
    uint8_t next_call_order = 1U;
    FakeModule first{0U, true, &next_call_order};
    FakeModule duplicate{0U, true, &next_call_order};
    FakeModule last{0U, true, &next_call_order};
    const std::array steps{
        app::InitializationStep{4U, "first", initialize_fake, &first},
        app::InitializationStep{4U, "duplicate", initialize_fake, &duplicate},
        app::InitializationStep{5U, "last", initialize_fake, &last},
    };
    TestContext context{};

    const app::InitializationReport report = app::InitializationManager{}.initialize_all(
        steps.data(), steps.size(), observe, &context);

    assert(first.call_order != 0U);
    assert(duplicate.call_order == 0U);
    assert(last.call_order != 0U);
    assert(context.observer_calls == steps.size());
    assert(!report.configuration_valid);
    assert(!report.all_succeeded());
}

} // namespace

int main()
{
    test_failure_does_not_short_circuit();
    test_duplicate_id_is_configuration_failure_but_iteration_continues();
    std::cout << "initialization manager tests passed\n";
    return 0;
}
