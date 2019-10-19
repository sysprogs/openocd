cmake_minimum_required (VERSION 3.0)
project (OpenOCD)

INCLUDE (CheckIncludeFiles)
SET(OPENOCD_VERSION 0.10.0)

include(config.cmake)

find_path(LIBUSB1_INCLUDE_DIR libusb.h
	HINTS ${CMAKE_SYSTEM_INCLUDE_PATH}
	PATH_SUFFIXES libusb-1.0)
	
find_path(HIDAPI_INCLUDE_DIR hidapi.h
	HINTS ${CMAKE_SYSTEM_INCLUDE_PATH}
	PATH_SUFFIXES hidapi)

find_library(LIBUSB1_LIBRARY usb-1.0 PATHS $ENV{TOOLCHAIN_ROOT}/msys/1.0/local/lib)
	
add_definitions(-DHAVE_CONFIG_H -DPKGDATADIR="../../share/openocd" -DBINDIR="../../bin" -D__USE_MINGW_ANSI_STDIO=1)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBUSB1_INCLUDE_DIR} ${HIDAPI_INCLUDE_DIR})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -Werror=incompatible-pointer-types")

add_subdirectory(jimtcl-cmake)
add_subdirectory(src)

INSTALL(DIRECTORY tcl/ DESTINATION share/openocd/scripts)
INSTALL(DIRECTORY contrib DESTINATION share/openocd)
