cmake_minimum_required (VERSION 3.0)
project (OpenOCD)

INCLUDE (CheckIncludeFiles)
SET(OPENOCD_VERSION 0.12.0)

set(SH_WRAPPER ${CMAKE_CURRENT_SOURCE_DIR}/external/run-sh.bat)
include(config.cmake)

if(HLA_MULTICORE)
set(HLA_MULTICORE_PREFIX "hla_multicore/")
set(HLA_MULTICORE_SUFFIX "_hla_multicore")
add_definitions(-DHLA_MULTICORE)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/hla_multicore)
else()
set(HLA_MULTICORE_PREFIX "")
set(HLA_MULTICORE_SUFFIX "")
endif()

add_subdirectory(external/hidapi/scripts)
add_subdirectory(external/libusb1/scripts)
add_subdirectory(external/prebuilt)

add_definitions(-DHAVE_CONFIG_H -DPKGDATADIR="../../share/openocd" -DBINDIR="../../bin" -D__USE_MINGW_ANSI_STDIO=1)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBUSB1_INCLUDE_DIR} ${HIDAPI_INCLUDE_DIR})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -Werror=incompatible-pointer-types")

add_subdirectory(jimtcl-cmake)
add_subdirectory(src)

INSTALL(DIRECTORY tcl/ DESTINATION share/openocd/scripts)
INSTALL(DIRECTORY contrib DESTINATION share/openocd)
