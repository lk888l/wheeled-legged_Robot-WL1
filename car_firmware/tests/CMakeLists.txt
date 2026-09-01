cmake_minimum_required(VERSION 3.20)
project(wl1_startup_tests LANGUAGES CXX)

enable_testing()

add_executable(initialization_manager_test
    initialization_manager_test.cpp
    ../Component/Application/InitializationManager.cpp
)
target_include_directories(initialization_manager_test PRIVATE
    ../Component/Application
)
target_compile_features(initialization_manager_test PRIVATE cxx_std_17)

add_test(NAME initialization_manager_test COMMAND initialization_manager_test)

add_executable(text_command_parser_test
    text_command_parser_test.cpp
)
target_include_directories(text_command_parser_test PRIVATE
    ../Component/Module/Basic
    ../Component/Module/include
)
target_compile_features(text_command_parser_test PRIVATE cxx_std_17)

add_test(NAME text_command_parser_test COMMAND text_command_parser_test)
