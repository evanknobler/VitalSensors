cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(VitalSensors)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/drivers.c)
target_sources(app PRIVATE src/wireless.c)

target_include_directories(app PRIVATE src/include)
