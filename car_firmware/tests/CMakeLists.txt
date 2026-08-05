cmake_minimum_required(VERSION 3.20)

project(wl1_host_tests LANGUAGES CXX)

add_executable(app_manager_tests
    app_manager_tests.cpp
    ../Component/App/src/app_manager.cpp
)
target_compile_features(app_manager_tests PRIVATE cxx_std_23)
target_compile_options(app_manager_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(app_manager_tests PRIVATE ../Component/App/include)

add_executable(control_algorithm_tests
    control_algorithm_tests.cpp
    ../Component/UserApp/CtrlAlgorithm/LQR.cpp
)
target_compile_features(control_algorithm_tests PRIVATE cxx_std_23)
target_compile_options(control_algorithm_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(control_algorithm_tests PRIVATE
    ../Component/UserApp/CtrlAlgorithm
)

enable_testing()
add_test(NAME app_manager_tests COMMAND app_manager_tests)
add_test(NAME control_algorithm_tests COMMAND control_algorithm_tests)
