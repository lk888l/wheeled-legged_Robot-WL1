#include "cpp_Interface.h"

#include "app_manager.hpp"
#include "app_modules.hpp"

extern "C" void CPP_Main(void)
{
    using wl1::app::AppManager;
    using namespace wl1::app_modules;

    initialize_runtime();

    static AppManager manager;
    if (!manager.register_module(&communication_module())
        || !manager.register_module(&servo_module())
        || !manager.register_module(&motion_control_module()))
    {
        report_startup_result(false);
        return;
    }

    report_startup_result(static_cast<bool>(manager.initialize_all()));
}
