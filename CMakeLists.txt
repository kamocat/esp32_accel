# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
	set(ENV{IDF_PATH} "$ENV{HOME}/.espressif/v5.5.4/esp-idf")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(captive_portal)

# Gzip static files into build directory for SPIFFS image
set(STATIC_GZ_DIR "${CMAKE_BINARY_DIR}/static_gz")

file(GLOB_RECURSE STATIC_SRCS
    RELATIVE "${CMAKE_SOURCE_DIR}/static"
    "${CMAKE_SOURCE_DIR}/static/*"
)

set(STATIC_GZ_OUTPUTS "")
foreach(REL_PATH ${STATIC_SRCS})
    if(NOT REL_PATH MATCHES "\\.gz$")
        set(SRC "${CMAKE_SOURCE_DIR}/static/${REL_PATH}")
        set(OUT "${STATIC_GZ_DIR}/${REL_PATH}.gz")
        get_filename_component(OUT_DIR "${OUT}" DIRECTORY)
        add_custom_command(
            OUTPUT "${OUT}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${OUT_DIR}"
            COMMAND ${CMAKE_COMMAND} -DINPUT=${SRC} -DOUTPUT=${OUT} -P ${CMAKE_SOURCE_DIR}/cmake/gzip_file.cmake
            DEPENDS "${SRC}"
            COMMENT "Gzipping ${REL_PATH}"
            VERBATIM
        )
        list(APPEND STATIC_GZ_OUTPUTS "${OUT}")
    endif()
endforeach()

add_custom_target(gzip_static_files ALL DEPENDS ${STATIC_GZ_OUTPUTS})

spiffs_create_partition_image(storage ${STATIC_GZ_DIR} FLASH_IN_PROJECT)
add_dependencies(spiffs_storage_bin gzip_static_files)
