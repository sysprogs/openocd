cmake_minimum_required (VERSION 3.0)
project (OpenOCD)

INCLUDE (CheckIncludeFiles)
SET(OPENOCD_VERSION 0.9.0)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

add_definitions(-DHAVE_CONFIG_H -DPKGDATADIR="." -DBINDIR="../../bin")
include_directories(${CMAKE_CURRENT_BINARY_DIR} C:/MinGW/include/libusb-1.0)
set(CMAKE_C_FLAGS "-std=gnu99")

add_subdirectory(jimtcl-cmake)
add_subdirectory(src)

INSTALL(DIRECTORY tcl/ DESTINATION share/openocd/scripts)
INSTALL(DIRECTORY contrib DESTINATION share/openocd)
