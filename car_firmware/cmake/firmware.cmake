include_guard(GLOBAL)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(wl1_project_options INTERFACE)
target_compile_options(wl1_project_options INTERFACE
    -mcpu=cortex-m4
    -mthumb
    -mthumb-interwork
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fmessage-length=0
    $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
)
target_compile_definitions(wl1_project_options INTERFACE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

add_library(wl1_strict_warnings INTERFACE)
target_compile_options(wl1_strict_warnings INTERFACE
    $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(wl1_project_options INTERFACE -Ofast)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(wl1_project_options INTERFACE -Ofast -g)
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    target_compile_options(wl1_project_options INTERFACE -Os)
else()
    target_compile_options(wl1_project_options INTERFACE -Og -g)
endif()

# CubeMX templates may supply generated-code glob results that include startup assembly.
# The final executable owns the startup file, so keep it out of the generated object target.
list(FILTER WL1_CUBEMX_SOURCES EXCLUDE REGEX "[Ss]tartup/.*\\.[sS]$")

add_library(wl1_stm32 OBJECT ${WL1_CUBEMX_SOURCES})
target_include_directories(wl1_stm32 PUBLIC
    ${WL1_CUBEMX_INCLUDE_DIRS}
    # freertos.c reaches the C++ application through this CubeMX USER CODE bridge.
    Component/UserApp
)
target_compile_definitions(wl1_stm32 PUBLIC ${WL1_CUBEMX_DEFINITIONS})
target_link_libraries(wl1_stm32 PUBLIC wl1_project_options)

add_subdirectory(Component)

set(WL1_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F411CEUX_FLASH.ld")
add_executable(${PROJECT_NAME}.elf
    Core/Startup/startup_stm32f411ceux.s
    ${WL1_LINKER_SCRIPT}
)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    wl1_user_app
    wl1_app_modules
    wl1_app
    wl1_control
    wl1_hardware
    wl1_peripheral
    wl1_base
    wl1_etl
    wl1_stm32
    wl1_project_options
)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mthumb-interwork
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -T${WL1_LINKER_SCRIPT}
    -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

set(HEX_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex")
set(BIN_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin")
find_program(WL1_OBJCOPY NAMES arm-none-eabi-objcopy REQUIRED)
find_program(WL1_SIZE NAMES arm-none-eabi-size REQUIRED)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${WL1_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${WL1_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMAND ${WL1_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

find_program(WL1_STLINK_CLI
    NAMES ST-LINK_CLI
    HINTS "C:/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility"
)
if(WL1_STLINK_CLI)
    add_custom_target(flash
        COMMAND ${WL1_STLINK_CLI} -c SWD UR -P ${HEX_FILE} -V after_programming -Rst
        DEPENDS ${PROJECT_NAME}.elf
        USES_TERMINAL
        COMMENT "Programming and verifying ${PROJECT_NAME}.hex with ST-Link"
    )
endif()
