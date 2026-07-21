file(GLOB U8G2_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/csrc/*.c")

add_library(u8g2 STATIC ${U8G2_SOURCES})
target_include_directories(u8g2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/csrc")
target_compile_definitions(u8g2 PUBLIC U8X8_WITH_USER_PTR)

if(NOT MSVC)
    target_compile_options(u8g2 PRIVATE
        -Wno-unused-parameter
        -Wno-sign-conversion
        -Wno-conversion
        -Wno-overlength-strings)
endif()
