# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(COMPONENTS main)
#set(CMAKE_C_COMPILER "/home/henla464/.espressif-master/tools/xtensa-esp32s3-elf-gcc11_2_0-esp-2022r1-RC1-linux-amd64/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc")
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/peripherals/usb/host/cdc)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(WiRoc-USBRadio)
