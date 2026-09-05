#include <array>
#include <string>
#include <vector>

#include "BoardHardware.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/TaskConfig.hpp"
#include "test_check.hpp"

extern "C" void CPP_Main(void);

namespace {

std::string failed_task;
bool inject_motion_fault{};

constexpr std::array<const char*, 8U> module_names{
    "command-uart", "imu-mpu6050", "left-encoder", "right-encoder",
    "wheel-motor", "left-servo", "right-servo", "radio-nrf24"};
constexpr std::array<const char*, 5U> task_names{
    "Heartbeat", "CommandService", "ServoControl", "MotionControl", "ButtonA0"};
constexpr std::array<configSTACK_DEPTH_TYPE, 5U> stack_depths{192U, 2000U, 256U, 2500U, 128U};
constexpr std::array<UBaseType_t, 5U> priorities{1U, 28U, 28U, 29U, 1U};

void check_initialization(bsp::HardwareModuleId)
{
    CHECK(fake_board::instance != nullptr);
    CHECK(fake_board::instance->safe_stops == 1U);
    CHECK(g_app_control_enabled == 0U);
    CHECK(g_app_system_state == static_cast<uint32_t>(app::SystemState::booting));
    CHECK(fake_rtos::creations.size() == 1U);
    CHECK(fake_rtos::creations.front().name == "Heartbeat");
    CHECK(fake_rtos::critical_depth == 0);
}

void enter_motion_fault()
{
    fake_rtos::on_resume = nullptr;
    // Model a higher-priority motion task failing before bootstrap has returned
    // from start(): the real RuntimeStatus must latch this transition.
    app::RuntimeStatus{}.enter_runtime_fault();
    fake_board::instance->force_safe_outputs();
}

bool create_task(const char* task_name)
{
    const std::string name{task_name};
    if (name == "Heartbeat") {
        CHECK(fake_board::instance->safe_stops == 1U);
        CHECK(fake_board::initializations.empty());
        CHECK(g_app_control_enabled == 0U);
    } else {
        CHECK(fake_board::initializations.size() == bsp::kHardwareModuleCount);
        CHECK(g_app_hardware_attempted_mask == bsp::kRequiredHardwareMask);
        CHECK(g_app_hardware_failed_mask == fake_board::failed_initialization_mask);
    }
    if (name == "CommandService" || name == "ServoControl") {
        CHECK(g_app_control_enabled == 0U);
    }
    if (name == "MotionControl") {
        CHECK(g_app_control_enabled == 1U);
        CHECK((g_app_task_attempted_mask & (1U << 2U)) != 0U);
        CHECK((g_app_task_failed_mask & (1U << 2U)) == 0U);
        if (inject_motion_fault) { fake_rtos::on_resume = enter_motion_fault; }
    }
    return name != failed_task;
}

std::string result_log(const char* category, const char* name, bool succeeded)
{
    return std::string{"["} + category + "][" + (succeeded ? " OK " : "FAIL") + "] " + name + '\n';
}

} // namespace

int main(int argc, char** argv)
{
    // CTest launches each scenario in a fresh process: CPP_Main owns objects
    // with static lifetime and is deliberately not a restartable API.
    fake_rtos::reset();
    fake_board::reset();
    std::vector<std::string> expected_tasks{"Heartbeat", "CommandService", "ServoControl", "MotionControl", "ButtonA0"};
    app::SystemState expected_state = app::SystemState::ready;
    bool expected_control = true;
    unsigned expected_safe_stops = 1U;
    uint32_t expected_task_failures = 0U;
    bool skip_command = false;
    const std::string scenario = argc > 1 ? argv[1] : "success";

    if (scenario == "hardware") {
        CHECK(argc == 3);
        const auto id = static_cast<unsigned>(std::stoul(argv[2]));
        CHECK(id < bsp::kHardwareModuleCount);
        fake_board::failed_initialization_mask = uint32_t{1} << id;
        expected_tasks = {"Heartbeat", "CommandService", "ButtonA0"};
        expected_state = app::SystemState::initialization_failed;
        expected_control = false;
        expected_safe_stops = 2U;
    } else if (scenario == "no-command-channel") {
        fake_board::failed_initialization_mask =
            (uint32_t{1} << bsp::module_id(bsp::HardwareModuleId::command_uart)) |
            (uint32_t{1} << bsp::module_id(bsp::HardwareModuleId::radio));
        expected_tasks = {"Heartbeat", "ButtonA0"};
        expected_state = app::SystemState::initialization_failed;
        expected_control = false;
        expected_safe_stops = 2U;
        skip_command = true;
    } else if (scenario == "task") {
        CHECK(argc == 3);
        const auto id = static_cast<unsigned>(std::stoul(argv[2]));
        CHECK(id < task_names.size());
        failed_task = task_names[id];
        expected_task_failures = uint32_t{1} << id;
        if (id < 2U) {
            expected_tasks = {"Heartbeat", "CommandService", "ButtonA0"};
        } else if (id == 2U) {
            expected_tasks = {"Heartbeat", "CommandService", "ServoControl", "ButtonA0"};
        }
        if (id != 4U) {
            expected_state = app::SystemState::task_failed;
            expected_control = false;
            expected_safe_stops = 2U;
        }
    } else if (scenario == "runtime-fault") {
        inject_motion_fault = true;
        expected_state = app::SystemState::runtime_fault;
        expected_control = false;
        expected_safe_stops = 2U;
    } else {
        CHECK(scenario == "success");
    }

    fake_board::on_initialize = check_initialization;
    fake_rtos::on_create = create_task;
    CPP_Main();

    const auto& board = *fake_board::instance;
    CHECK(g_app_hardware_attempted_mask == bsp::kRequiredHardwareMask);
    CHECK(g_app_hardware_failed_mask == fake_board::failed_initialization_mask);
    CHECK(g_app_task_failed_mask == expected_task_failures);
    CHECK(g_app_system_state == static_cast<uint32_t>(expected_state));
    CHECK((g_app_control_enabled != 0U) == expected_control);
    CHECK(board.safe_stops == expected_safe_stops);
    CHECK(fake_rtos::critical_depth == 0 && fake_rtos::scheduler_depth == 0U);

    CHECK(fake_board::initializations.size() == bsp::kHardwareModuleCount);
    for (size_t i = 0U; i < bsp::kHardwareModuleCount; ++i) {
        CHECK(bsp::module_id(fake_board::initializations[i]) == i);
        CHECK(fake_board::initialization_ticks[i] == pdMS_TO_TICKS(3U * (i + 1U)));
    }

    CHECK(fake_rtos::creations.size() == expected_tasks.size());
    uint32_t expected_attempts = 0U;
    for (size_t i = 0U; i < expected_tasks.size(); ++i) {
        const auto& creation = fake_rtos::creations[i];
        CHECK(creation.name == expected_tasks[i]);
        size_t id = 0U;
        while (id < task_names.size() && creation.name != task_names[id]) { ++id; }
        CHECK(id < task_names.size());
        expected_attempts |= uint32_t{1} << id;
        CHECK(creation.stack_depth == stack_depths[id]);
        CHECK(creation.priority == priorities[id]);
        CHECK(creation.uses_static_storage == (id == 4U));
        CHECK(creation.tick == (i == 0U ? 0U : pdMS_TO_TICKS(3U * (8U + i))));
    }
    CHECK(g_app_task_attempted_mask == expected_attempts);
    CHECK(fake_rtos::static_creates == 1U);
    CHECK(fake_rtos::dynamic_creates == expected_tasks.size() - 1U);

    // The real entry point must still report every result in declaration order
    // and yield between bursts so startup does not overflow the UART buffer.
    CHECK(fake_rtos::delays.size() == bsp::kHardwareModuleCount + expected_tasks.size());
    for (const auto delay : fake_rtos::delays) { CHECK(delay == pdMS_TO_TICKS(3U)); }
    const auto& logs = fake_board::instance->command_uart().logs;
    size_t line = 0U;
    CHECK(logs.size() == 2U + bsp::kHardwareModuleCount + expected_tasks.size() + (skip_command ? 1U : 0U));
    CHECK(logs[line++] == "[app] WL1 startup begin\n");
    CHECK(logs[line++] == result_log("task", "Heartbeat", failed_task != "Heartbeat"));
    for (size_t i = 0U; i < module_names.size(); ++i) {
        CHECK(logs[line++] == result_log("init", module_names[i],
              (fake_board::failed_initialization_mask & (uint32_t{1} << i)) == 0U));
    }
    if (skip_command) { CHECK(logs[line++] == "[task][SKIP] CommandService: no command channel\n"); }
    for (size_t i = 1U; i < expected_tasks.size(); ++i) {
        CHECK(logs[line++] == result_log("task", expected_tasks[i].c_str(), expected_tasks[i] != failed_task));
    }
    CHECK(logs[line].find(std::string{"[app] state="} + app::system_state_name(expected_state)) == 0U);

    // Repeated late bootstrap writes must never recover a runtime fault.
    if (inject_motion_fault) {
        app::RuntimeStatus status;
        status.set_state(app::SystemState::ready);
        status.enable_control(true);
        CHECK(status.state() == app::SystemState::runtime_fault);
        CHECK(!status.control_enabled());
    }
    std::cout << "startup scenario passed: " << scenario << '\n';
}
