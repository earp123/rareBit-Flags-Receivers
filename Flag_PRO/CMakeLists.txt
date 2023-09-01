
# CMake min version
cmake_minimum_required(VERSION 3.8.2)

# Include Zephyr base folder
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_peripheral_simple)

# Source files
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/pulse_rgb.c)
target_sources(app PRIVATE src/battery.c)

target_include_directories(app PRIVATE src/inc)
