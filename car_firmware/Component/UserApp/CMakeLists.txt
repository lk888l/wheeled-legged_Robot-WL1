add_library(wl1_control OBJECT
    CtrlAlgorithm/JumpController.cpp
    CtrlAlgorithm/LegKinematics.cpp
    CtrlAlgorithm/LQR.cpp
    CtrlAlgorithm/PID.cpp
)
target_include_directories(wl1_control PUBLIC CtrlAlgorithm)
target_link_libraries(wl1_control PUBLIC
    wl1_project_options
)
target_link_libraries(wl1_control PRIVATE wl1_strict_warnings)

add_library(wl1_user_app OBJECT main.cpp)
target_include_directories(wl1_user_app PUBLIC .)
target_link_libraries(wl1_user_app PUBLIC
    wl1_app
    wl1_app_modules
    wl1_stm32
    wl1_project_options
)
target_link_libraries(wl1_user_app PRIVATE wl1_strict_warnings)
