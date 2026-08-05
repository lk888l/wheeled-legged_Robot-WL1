add_library(wl1_hardware OBJECT
    IMU/MPU6050.cpp
    IMU/vqf.cpp
    Motor/HallEncoder.cpp
    Motor/Servo.cpp
    Motor/TB6612.cpp
    OLED/HAL_OLED.c
    OLED/OLED_Data.c
    Telecontrol/NRF24L01P.cpp
)
target_include_directories(wl1_hardware PUBLIC
    IMU
    Motor
    OLED
    Telecontrol
)
target_link_libraries(wl1_hardware PUBLIC
    wl1_base
    wl1_etl
    wl1_stm32
    wl1_project_options
)
