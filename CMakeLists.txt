# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
	set(ENV{IDF_PATH} "$ENV{HOME}/.espressif/v5.5.4/esp-idf")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(captive_portal)
