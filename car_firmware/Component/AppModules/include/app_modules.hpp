#pragma once

#include "app_module.hpp"

namespace wl1::app_modules {

app::AppModule& communication_module();
app::AppModule& servo_module();
app::AppModule& motion_control_module();

void initialize_runtime();
void report_startup_result(bool success);

} // namespace wl1::app_modules
