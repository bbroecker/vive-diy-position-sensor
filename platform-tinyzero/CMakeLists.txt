cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_CXX_STANDARD 17)
#set(ARDUINO_INSTALL_PATH /home/broecker/arduino-1.8.12)
#set(CMAKE_TOOLCHAIN_FILE /home/broecker/vive-diy-position-sensor/libs/Arduino-CMake-Toolchain/Arduino-toolchain.cmake)


project(tiny_main CXX)


set(TINY_SOURCE_FILES
        main.cpp
        timestamp.cpp
        car_control/bluetooth_handler.cpp
        car_control/motor_controller.cpp
        input_tim_2.cpp
        samd21_timer.cpp
        samd21_timer.h
        i2c_communication.h
        i2c_communication.cpp
)

set(TINY_MOTOR_FILES
    motor_main.cpp
    timestamp.cpp
    car_control/bluetooth_handler.cpp
    car_control/motor_controller.cpp
    UART.h)

set(TC3_FILES
    timer_test.cpp
    samd21_timer.h
    samd21_timer.cpp)

add_executable(tiny_main "${TINY_SOURCE_FILES}")
add_executable(tiny_motor "${TINY_MOTOR_FILES}")
add_executable(tc_main "${TC3_FILES}")


target_link_libraries(tiny_main PRIVATE sensor-core)
target_link_libraries(tiny_motor PRIVATE sensor-core)
target_link_libraries(tc_main PRIVATE sensor-core)

# Arduino.h is included in hello_world.cpp, so link with Arduino core
target_link_arduino_libraries(tiny_main PRIVATE core SPI STBLE Wire)
target_link_arduino_libraries(tiny_motor PRIVATE core SPI STBLE)
target_link_arduino_libraries(tc_main PRIVATE core SPI STBLE)

# This is needed for the generation of HEX binary and uploading it
target_enable_arduino_upload(tiny_main)
target_enable_arduino_upload(tiny_motor)
target_enable_arduino_upload(tc_main)

message(${CMAKE_SOURCE_DIR})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/reset_serial.py
    ${CMAKE_BINARY_DIR}/reset_serial.py COPYONLY)

#add_custom_target(sensor-firmware-upload
# COMMAND ${CMAKE_SOURCE_DIR}/platform-tinyzero/scripts/reset_serial.py /dev/ttyACM0
# COMMAND make SERIAL_PORT=ttyACM0 SERIAL_PORT_FILE=ttyACM0  upload-tiny_main)

