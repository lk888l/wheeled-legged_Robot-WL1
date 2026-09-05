cmake_minimum_required(VERSION 3.20)
project(wl1_application_tests LANGUAGES CXX)

enable_testing()

# Legacy tests use assert(); keep checks active in optimized host builds too.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -UNDEBUG)
elseif(MSVC)
    add_compile_options(/UNDEBUG)
endif()

add_executable(initialization_report_test
    initialization_report_test.cpp
)
target_include_directories(initialization_report_test PRIVATE
    ../Component/Application
)
target_compile_features(initialization_report_test PRIVATE cxx_std_17)

add_test(NAME initialization_report_test COMMAND initialization_report_test)

add_executable(text_command_parser_test
    text_command_parser_test.cpp
)
target_include_directories(text_command_parser_test PRIVATE
    ../Component/Module/Basic
    ../Component/Module/include
)
target_compile_features(text_command_parser_test PRIVATE cxx_std_17)

add_test(NAME text_command_parser_test COMMAND text_command_parser_test)

function(add_wl1_test name)
    add_executable(${name} ${name}.cpp ${ARGN})
    target_include_directories(${name} PRIVATE
        ../Component/Application ../Component/Bsp ../Component/UserApp
        ../Component/Module/Basic ../Component/Module/include)
    target_compile_features(${name} PRIVATE cxx_std_17)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 20)
endfunction()

add_wl1_test(button_test ../Component/Application/Button.cpp)
add_wl1_test(button_event_queue_test)
find_package(Threads REQUIRED)
target_link_libraries(button_event_queue_test PRIVATE Threads::Threads)
add_wl1_test(telemetry_frame_test)
add_wl1_test(pid_test)
add_wl1_test(app_task_test ../Component/Application/AppTask.cpp)
add_wl1_test(task_reactor_test)
add_wl1_test(runtime_state_test
    ../Component/Application/RuntimeStatus.cpp ../Component/UserApp/ControlState.cpp)
add_wl1_test(button_task_test
    ../Component/Application/AppTask.cpp ../Component/Application/Button.cpp
    ../Component/UserApp/Tasks/ButtonTask.cpp)
add_wl1_test(command_service_test
    ../Component/Application/AppTask.cpp ../Component/Application/Button.cpp
    ../Component/Application/RuntimeStatus.cpp ../Component/UserApp/ControlState.cpp
    ../Component/UserApp/Tasks/ButtonTask.cpp ../Component/UserApp/Tasks/CommandServiceTask.cpp)
add_wl1_test(motion_control_task_test
    ../Component/Application/AppTask.cpp ../Component/Application/RuntimeStatus.cpp
    ../Component/UserApp/ControlState.cpp ../Component/UserApp/Tasks/MotionControlTask.cpp)
add_wl1_test(startup_test
    ../Component/Application/AppTask.cpp ../Component/Application/Button.cpp
    ../Component/Application/RuntimeStatus.cpp ../Component/UserApp/ControlState.cpp
    ../Component/UserApp/main.cpp
    ../Component/UserApp/Tasks/HeartbeatTask.cpp ../Component/UserApp/Tasks/ButtonTask.cpp
    ../Component/UserApp/Tasks/CommandServiceTask.cpp ../Component/UserApp/Tasks/ServoControlTask.cpp
    ../Component/UserApp/Tasks/MotionControlTask.cpp)
# Each process calls the real static-lifetime CPP_Main exactly once.
foreach(module RANGE 0 7)
    add_test(NAME startup_hardware_failure_${module} COMMAND startup_test hardware ${module})
    set_tests_properties(startup_hardware_failure_${module} PROPERTIES TIMEOUT 20)
endforeach()
foreach(task RANGE 0 4)
    add_test(NAME startup_task_failure_${task} COMMAND startup_test task ${task})
    set_tests_properties(startup_task_failure_${task} PROPERTIES TIMEOUT 20)
endforeach()
add_test(NAME startup_no_command_channel COMMAND startup_test no-command-channel)
add_test(NAME startup_runtime_fault COMMAND startup_test runtime-fault)
set_tests_properties(startup_no_command_channel startup_runtime_fault PROPERTIES TIMEOUT 20)

foreach(name app_task_test task_reactor_test runtime_state_test button_task_test command_service_test motion_control_task_test startup_test)
    target_include_directories(${name} BEFORE PRIVATE fakes)
endforeach()
