add_subdirectory(autosetup)

set(jimtcl_generated_sources binary.c glob.c nshelper.c oo.c stdlib.c tclcompat.c tree.c)

foreach(jim_src ${jimtcl_generated_sources} initjimsh.c)
	get_filename_component (src_base ${jim_src} NAME_WE)
	
	add_custom_command(OUTPUT ${jim_src} COMMAND ${CMAKE_CURRENT_BINARY_DIR}/autosetup/jimsh0.exe ARGS ${CMAKE_CURRENT_SOURCE_DIR}/../jimtcl/make-c-ext.tcl ${CMAKE_CURRENT_SOURCE_DIR}/../jimtcl/${src_base}.tcl > ${jim_src} MAIN_DEPENDENCY ../jimtcl/${src_base}.tcl DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/autosetup/jimsh0.exe)
endforeach(jim_src)

INCLUDE (CheckIncludeFiles)
SET(JIM_VERSION 76)

if (WIN32)
    set(TCL_PLATFORM_OS "mingw")
    set(TCL_PLATFORM_PATH_SEPARATOR ";")
    set(TCL_PLATFORM_PLATFORM "windows")
endif (WIN32)

if (UNIX)
    set(TCL_PLATFORM_OS "linux")
    set(TCL_PLATFORM_PATH_SEPARATOR ":")
    set(TCL_PLATFORM_PLATFORM "unix")
endif (UNIX)


CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/jim-config.h.in ${CMAKE_CURRENT_BINARY_DIR}/jim-config.h)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/jimautoconf.h.in ${CMAKE_CURRENT_BINARY_DIR}/jimautoconf.h)

add_custom_command(OUTPUT _load-static-exts.c COMMAND ${CMAKE_CURRENT_BINARY_DIR}/autosetup/jimsh0.exe ARGS ${CMAKE_CURRENT_SOURCE_DIR}/../jimtcl/make-load-static-exts.tcl aio array clock eventloop exec file history load namespace pack package readdir regexp tclprefix binary glob nshelper oo stdlib tclcompat tree > _load-static-exts.c MAIN_DEPENDENCY ../jimtcl/make-load-static-exts.tcl DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/autosetup/jimsh0.exe)

include_directories(../jimtcl ${CMAKE_CURRENT_BINARY_DIR})
add_library(jim STATIC ${jimtcl_generated_sources} 
	../jimtcl/jim-subcmd.c
	../jimtcl/jim-interactive.c
	../jimtcl/jim-format.c
	../jimtcl/jim.c
	../jimtcl/utf8.c
	../jimtcl/jimregexp.c
	../jimtcl/linenoise.c
	../jimtcl/jim-win32compat.c
	../jimtcl/jim-aio.c
	../jimtcl/jim-array.c
	../jimtcl/jim-clock.c
	../jimtcl/jim-eventloop.c
	../jimtcl/jim-exec.c
	../jimtcl/jim-file.c
	../jimtcl/jim-history.c
	../jimtcl/jim-load.c
	../jimtcl/jim-namespace.c
	../jimtcl/jim-pack.c
	../jimtcl/jim-package.c
	../jimtcl/jim-readdir.c
	../jimtcl/jim-regexp.c
	../jimtcl/jim-tclprefix.c
	../jimtcl/jim-nosignal.c
	../jimtcl/jimiocompat.c
	_load-static-exts.c)
	
add_executable(jimsh ../jimtcl/jimsh.c initjimsh.c)
target_link_libraries(jimsh jim)