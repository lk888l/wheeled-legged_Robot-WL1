add_library(wl1_app_modules OBJECT
    src/communication_module.cpp
    src/motion_control_module.cpp
    src/runtime.cpp
    src/servo_module.cpp
)
target_include_directories(wl1_app_modules
    PUBLIC include
    PRIVATE src
)
target_link_libraries(wl1_app_modules PUBLIC
    wl1_app
    wl1_base
    wl1_control
    wl1_hardware
    wl1_peripheral
    wl1_etl
    wl1_stm32
    wl1_project_options
)
target_link_libraries(wl1_app_modules PRIVATE wl1_strict_warnings)
