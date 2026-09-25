#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/TaskConfig.hpp"
#include "test_check.hpp"

int main()
{
    fake_rtos::reset();
    app::ControlState control;
    auto parameters = control.parameters();
    CHECK(parameters.leg_height == 44.5F && parameters.angle.kp == 75.35F);
    CHECK(parameters.roll_bias == 0.0F);
    parameters.velocity_target = 10.0F;
    parameters.difference_target = 20.0F;
    parameters.leg_height = 55.0F;
    parameters.roll_target = 5.0F;
    parameters.roll_bias = -2.5F;
    control.set_parameters(parameters);
    auto read = control.parameters();
    CHECK(read.velocity_target == 10.0F && read.difference_target == 20.0F);
    CHECK(read.leg_height == 55.0F && read.roll_target == 5.0F);
    CHECK(read.roll_bias == -2.5F);
    read.leg_height = 70.0F;
    CHECK(control.parameters().leg_height == 55.0F);
    control.publish_leg_targets({44.5F, 78.5F});
    CHECK(control.leg_targets().right == 78.5F);
    control.set_installation_mode(true);
    CHECK(control.parameters().roll_target == 0.0F && control.parameters().roll_bias == -2.5F);
    parameters.roll_target = 7.0F;
    control.set_parameters(parameters);
    CHECK(control.parameters().roll_target == 0.0F && control.parameters().roll_bias == -2.5F);
    control.set_installation_mode(false);
    CHECK(control.parameters().roll_bias == -2.5F);
    CHECK(fake_rtos::critical_depth == 0 && fake_rtos::critical_entries > 0U);

    app::RuntimeStatus status;
    status.reset();
    status.enable_control(true);
    status.set_state(app::SystemState::ready);
    status.record_task_result(app::TaskId::button, false);
    CHECK(status.task_failed_mask() == (1U << 4U));
    status.record_task_result(static_cast<app::TaskId>(32U), false);
    CHECK(status.task_failed_mask() == (1U << 4U));
    CHECK((status.task_failed_mask() & app::task_config::required_task_mask) == 0U);
    CHECK(status.control_enabled());
    status.enter_runtime_fault();
    status.set_state(app::SystemState::ready); // Late bootstrap must not undo fault.
    status.enable_control(true);
    CHECK(status.state() == app::SystemState::runtime_fault && !status.control_enabled());
    CHECK(fake_rtos::critical_depth == 0);
}
