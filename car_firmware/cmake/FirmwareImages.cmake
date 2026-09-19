# One application, two installation policies. FactoryDefaults uses the target
# compiler and the runtime record encoder; no second copy of tuning constants.
add_library(wl1_factory_defaults OBJECT ${CMAKE_CURRENT_LIST_DIR}/../tools/FactoryDefaults.cpp)
set(image_prefix "${PROJECT_BINARY_DIR}/${PROJECT_NAME}")
set(image_outputs
    "${image_prefix}.hex" "${image_prefix}.bin"
    "${image_prefix}_update.hex" "${image_prefix}_update.bin"
    "${image_prefix}_factory.hex" "${image_prefix}_factory.bin"
    "${image_prefix}_defaults.bin")

# OUTPUT dependencies regenerate missing images and rebuild when defaults change,
# even when the application ELF itself has not needed relinking.
add_custom_command(OUTPUT ${image_outputs}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> "${image_prefix}_update.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary --gap-fill 0xFF $<TARGET_FILE:${PROJECT_NAME}.elf> "${image_prefix}_update.bin"
    COMMAND ${CMAKE_OBJCOPY} -O binary --only-section=.motion_defaults
        $<TARGET_OBJECTS:wl1_factory_defaults> "${image_prefix}_defaults.bin"
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        --add-section ".motion_defaults=${image_prefix}_defaults.bin"
        --set-section-flags .motion_defaults=alloc,load,readonly,data
        --change-section-address .motion_defaults=0x08060000
        $<TARGET_FILE:${PROJECT_NAME}.elf> "${image_prefix}_factory.hex"
    COMMAND ${CMAKE_OBJCOPY} -I ihex -O binary --gap-fill 0xFF
        "${image_prefix}_factory.hex" "${image_prefix}_factory.bin"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${image_prefix}_update.hex" "${image_prefix}.hex"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${image_prefix}_update.bin" "${image_prefix}.bin"
    DEPENDS ${PROJECT_NAME}.elf wl1_factory_defaults $<TARGET_OBJECTS:wl1_factory_defaults>
        "${CMAKE_CURRENT_LIST_FILE}"
    COMMENT "Building update (preserve parameters) and factory (default parameters) images"
    VERBATIM)
add_custom_target(firmware_images ALL DEPENDS ${image_outputs})

set(WL1_OPENOCD_CONFIG "${PROJECT_SOURCE_DIR}/STlink.cfg" CACHE FILEPATH "OpenOCD probe configuration")
find_program(WL1_OPENOCD_EXECUTABLE NAMES openocd)
foreach(image_mode IN ITEMS update factory)
    # Explicitly erase all application sectors. Factory additionally erases the
    # WHOLE journal: an old valid record after slot zero must not survive.
    if(image_mode STREQUAL "factory")
        set(image_last_sector 7)
    else()
        set(image_last_sector 6)
    endif()
    configure_file("${CMAKE_CURRENT_LIST_DIR}/flash-image.cfg.in"
        "${PROJECT_BINARY_DIR}/flash_${image_mode}.cfg" @ONLY)
    if(WL1_OPENOCD_EXECUTABLE)
        add_custom_target(flash_${image_mode}
            COMMAND "${WL1_OPENOCD_EXECUTABLE}" -f "${WL1_OPENOCD_CONFIG}"
                -f "${PROJECT_BINARY_DIR}/flash_${image_mode}.cfg"
            DEPENDS firmware_images
            COMMENT "Flashing ${image_mode} image (erase sectors 0..${image_last_sector})"
            USES_TERMINAL VERBATIM)
    endif()
endforeach()
