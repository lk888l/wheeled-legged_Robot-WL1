cmake_minimum_required(VERSION 3.22)

# This file is generated only once by STM32CubeMX and is intended to be edited.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_PROJECT_NAME WL1_F411CEU6_Tele)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

enable_language(C CXX ASM)
add_executable(${CMAKE_PROJECT_NAME})

# STM32CubeMX generated sources, include paths, definitions and middleware.
add_subdirectory(cmake/stm32cubemx)

# Application and board-support code maintained outside the CubeMX generated tree.
# Keep this list explicit so obsolete or experimental sources are not linked silently.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Component/HardWare/OLED/U8g2Oled.cpp
    Component/HardWare/Telecontrol/NRF24L01P.cpp
    Component/Peripheral/Button.cpp
    Component/Peripheral/LkUart.cpp
    Component/UserApp/RemoteCommandCodec.cpp
    Component/UserApp/RemoteControlState.cpp
    Component/UserApp/RemoteDisplay.cpp
    Component/UserApp/SerialCommandBridge.cpp
    Component/UserApp/main.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Component/UserApp
    Component/HardWare/OLED
    Component/HardWare/Telecontrol
    Component/Peripheral
    Component/Module/include
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -fno-common
    -fmessage-length=0
)

# The remote-control packet is formatted with snprintf() and contains floats.
# nano.specs omits floating-point printf support unless this symbol is forced in.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

add_subdirectory(ThirdParty/u8g2)

# Remove an invalid implicit library that can be injected by some toolchains.
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    u8g2
)

set(HEX_FILE "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex")
set(BIN_FILE "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
    COMMENT "Generating ${HEX_FILE} and ${BIN_FILE}"
)
