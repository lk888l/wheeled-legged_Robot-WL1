add_library(wl1_etl INTERFACE)
target_include_directories(wl1_etl SYSTEM INTERFACE include)

add_library(wl1_base OBJECT
    Basic/SafeQueue.cpp
    Basic/TaskReactor.cpp
)
target_include_directories(wl1_base PUBLIC Basic)
target_link_libraries(wl1_base PUBLIC
    wl1_etl
    wl1_stm32
    wl1_project_options
)
target_link_libraries(wl1_base PRIVATE wl1_strict_warnings)
