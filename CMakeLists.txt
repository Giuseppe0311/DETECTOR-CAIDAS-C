# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")
set(PARTITION_TABLE_CUSTOM_APP_BIN_OFFSET "0x10000")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cuidado_abu)
