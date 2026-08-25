cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3")

# Build only this test application and the components it actually depends on.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wheel-test)
