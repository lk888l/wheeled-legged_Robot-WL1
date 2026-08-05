#include <array>
#include <cassert>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "app_manager.hpp"

namespace {

using wl1::app::AppManager;
using wl1::app::AppModule;
using wl1::app::LifecycleStatus;
using wl1::app::RegistrationStatus;

class FakeModule final : public AppModule
{
public:
    FakeModule(std::string_view name,
               std::vector<std::string>& calls,
               bool initialize_succeeds = true,
               bool cleanup_succeeds = true)
        : name_(name),
          calls_(calls),
          initialize_succeeds_(initialize_succeeds),
          cleanup_succeeds_(cleanup_succeeds)
    {
    }

    [[nodiscard]] std::string_view name() const override
    {
        return name_;
    }

    void process() override
    {
        calls_.emplace_back("process:" + std::string(name_));
    }

private:
    bool on_initialize() override
    {
        calls_.emplace_back("start:" + std::string(name_));
        return initialize_succeeds_;
    }

    bool on_deinitialize() override
    {
        calls_.emplace_back("stop:" + std::string(name_));
        return cleanup_succeeds_;
    }

    std::string_view name_;
    std::vector<std::string>& calls_;
    bool initialize_succeeds_;
    bool cleanup_succeeds_;
};

void test_registration_validation()
{
    AppManager manager;
    std::vector<std::string> calls;
    std::array<FakeModule, 9> modules{
        FakeModule{"module-0", calls},
        FakeModule{"module-1", calls},
        FakeModule{"module-2", calls},
        FakeModule{"module-3", calls},
        FakeModule{"module-4", calls},
        FakeModule{"module-5", calls},
        FakeModule{"module-6", calls},
        FakeModule{"module-7", calls},
        FakeModule{"overflow", calls},
    };
    FakeModule duplicate{"module-0", calls};

    assert(manager.register_module(nullptr).status == RegistrationStatus::null_module);
    assert(manager.register_module(&modules[0]));
    assert(manager.register_module(&duplicate).status == RegistrationStatus::duplicate_name);
    for (std::size_t index = 1; index < AppManager::kMaxModules; ++index)
    {
        assert(manager.register_module(&modules[index]));
    }
    assert(manager.module_count() == AppManager::kMaxModules);
    assert(manager.register_module(&modules.back()).status == RegistrationStatus::registry_full);
}

void test_initialization_rollback_order()
{
    AppManager manager;
    std::vector<std::string> calls;
    FakeModule first{"first", calls};
    FakeModule second{"second", calls};
    FakeModule failure{"failure", calls, false};
    assert(manager.register_module(&first));
    assert(manager.register_module(&second));
    assert(manager.register_module(&failure));

    const auto result = manager.initialize_all();
    assert(result.status == LifecycleStatus::module_failed);
    assert(result.module_name == "failure");
    const std::vector<std::string> expected{
        "start:first",
        "start:second",
        "start:failure",
        "stop:failure",
        "stop:second",
        "stop:first",
    };
    assert(calls == expected);
    assert(!manager.is_running());
}

void test_failed_cleanup_faults_manager()
{
    AppManager manager;
    std::vector<std::string> calls;
    FakeModule dependency{"dependency", calls};
    FakeModule failure{"failure", calls, false, false};
    assert(manager.register_module(&dependency));
    assert(manager.register_module(&failure));

    const auto result = manager.initialize_all();
    assert(result.status == LifecycleStatus::rollback_failed);
    assert(result.module_name == "failure");
    assert(manager.has_cleanup_failure());
    const std::vector<std::string> expected{
        "start:dependency",
        "start:failure",
        "stop:failure",
    };
    assert(calls == expected);
}

void test_process_and_reverse_stop()
{
    AppManager manager;
    std::vector<std::string> calls;
    FakeModule first{"first", calls};
    FakeModule second{"second", calls};
    FakeModule too_late{"too-late", calls};
    assert(manager.register_module(&first));
    assert(manager.register_module(&second));
    assert(manager.initialize_all());
    assert(manager.register_module(&too_late).status == RegistrationStatus::invalid_state);

    manager.process_all();
    assert(manager.deinitialize_all());
    manager.process_all();
    const std::vector<std::string> expected{
        "start:first",
        "start:second",
        "process:first",
        "process:second",
        "stop:second",
        "stop:first",
    };
    assert(calls == expected);
}

} // namespace

int main()
{
    test_registration_validation();
    test_initialization_rollback_order();
    test_failed_cleanup_faults_manager();
    test_process_and_reverse_stop();
    std::cout << "all app manager tests passed\n";
}
