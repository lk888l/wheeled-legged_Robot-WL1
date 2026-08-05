add_library(wl1_peripheral OBJECT LkUart.cpp)
target_include_directories(wl1_peripheral PUBLIC .)
target_link_libraries(wl1_peripheral PUBLIC
    wl1_base
    wl1_etl
    wl1_stm32
    wl1_project_options
)
target_link_libraries(wl1_peripheral PRIVATE wl1_strict_warnings)
