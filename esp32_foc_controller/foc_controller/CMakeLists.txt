# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# 新版双电机驱动板使用 ESP32-S3；仍允许命令行显式覆盖目标。
if(NOT DEFINED IDF_TARGET)
    set(IDF_TARGET "esp32s3")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(foc_controller)
