add_library(wl1_app OBJECT src/app_manager.cpp)
target_include_directories(wl1_app PUBLIC include)
target_link_libraries(wl1_app PUBLIC
    wl1_project_options
)
target_link_libraries(wl1_app PRIVATE wl1_strict_warnings)
