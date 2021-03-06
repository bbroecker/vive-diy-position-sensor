cmake_minimum_required(VERSION 3.2)

if (${PLATFORM} MATCHES "Teensy")
    set(CMAKE_TOOLCHAIN_FILE "platform-teensy/teensy-arm.toolchain.cmake")
    project(vive-diy-position-sensor)
    add_subdirectory(platform-teensy)
    add_subdirectory(src)

elseif (${PLATFORM} MATCHES "Particle")
    set(CMAKE_TOOLCHAIN_FILE "platform-particle/stm32f2xx.toolchain.cmake")
    project(vive-diy-position-sensor)
    add_subdirectory(platform-particle)
    add_subdirectory(src)
elseif (${PLATFORM} MATCHES "Tiny")
    set(ARDUINO_BOARD "TinyZero [samd.tinyzero]") # TinyZero
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/libs/Arduino-CMake-Toolchain/Arduino-toolchain.cmake)
    include("platform-tinyzero/tiny-zero-arm.toolchain.cmake")
    project(vive-diy-position-sensor)
    add_subdirectory(platform-tinyzero)
    add_subdirectory(src)
elseif (${PLATFORM} MATCHES "Seeduino")
    set(ARDUINO_BOARD "Seeeduino XIAO [samd.seeed_XIAO_m0]") #
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/libs/Arduino-CMake-Toolchain/Arduino-toolchain.cmake)
    include("platform-tinyzero/tiny-zero-arm.toolchain.cmake")
    project(vive-diy-position-sensor)
    add_subdirectory(platform-tinyzero)
    add_subdirectory(src)

elseif (${PLATFORM} MATCHES "Host_Test")
    # Need to define project at the top level to be able to call ctest from build directory
    set(CMAKE_TOOLCHAIN_FILE "test/32bit.toolchain.cmake")
    project(vive-diy-position-sensor)
    include(CTest)
    add_subdirectory(test)
    add_subdirectory(src)

else()
    message(FATAL_ERROR "Please provide PLATFORM env variable")

endif()
