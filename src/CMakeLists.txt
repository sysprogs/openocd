include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/helper ${CMAKE_CURRENT_SOURCE_DIR}/../jimtcl ${CMAKE_CURRENT_BINARY_DIR}/../jimtcl-cmake ${CMAKE_CURRENT_BINARY_DIR})

add_library(helper helper/binarybuffer.c helper/options.c helper/time_support_common.c helper/configuration.c helper/log.c helper/command.c helper/time_support.c helper/replacements.c helper/fileio.c helper/util.c helper/jim-nvp.c helper/jep106.c helper/unicode.c helper/crc32.c helper/nvp.c helper/base64.c)

add_library(transport transport/transport.c)
add_library(svf svf/svf.c)
add_library(xsvf xsvf/xsvf.c)
add_library(pld pld/pld.c pld/xilinx_bit.c pld/virtex2.c pld/efinix.c pld/certus.c pld/ecp2_3.c pld/ecp5.c pld/gatemate.c pld/gowin.c pld/intel.c pld/lattice.c pld/lattice_bit.c pld/raw_bit.c)
add_library(rtt rtt/rtt.c rtt/tcl.c)
add_library(server server/server.c server/telnet_server.c server/gdb_server.c server/rtt_server.c server/tcl_server.c server/ipdbg.c)
add_library(rtos rtos/rtos.c rtos/rtos_standard_stackings.c rtos/rtos_ecos_stackings.c rtos/rtos_chibios_stackings.c rtos/rtos_embkernel_stackings.c rtos/rtos_mqx_stackings.c rtos/FreeRTOS.c rtos/ThreadX.c rtos/eCos.c rtos/linux.c rtos/ChibiOS.c rtos/embKernel.c rtos/mqx.c rtos/rtos_ucos_iii_stackings.c rtos/nuttx.c rtos/chromium-ec.c rtos/hwthread.c rtos/riot.c rtos/rtos_riot_stackings.c rtos/zephyr.c rtos/rtkernel.c rtos/rtos_nuttx_stackings.c rtos/ucos_iii.c)

add_subdirectory(target)
add_subdirectory(jtag)
add_subdirectory(flash)

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER>  <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> -Wl,--start-group <LINK_LIBRARIES> -Wl,--end-group")

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/startup.tcl COMMAND ${SH_WRAPPER} ARGS -c "cat ${CMAKE_CURRENT_SOURCE_DIR}/helper/startup.tcl ${CMAKE_CURRENT_SOURCE_DIR}/jtag/startup.tcl ${CMAKE_CURRENT_SOURCE_DIR}/target/startup.tcl ${CMAKE_CURRENT_SOURCE_DIR}/flash/startup.tcl ${CMAKE_CURRENT_SOURCE_DIR}/server/startup.tcl > ${CMAKE_CURRENT_BINARY_DIR}/startup.tcl" MAIN_DEPENDENCY helper/startup.tcl)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/startup_tcl.inc COMMAND ${SH_WRAPPER} ARGS -c "${CMAKE_CURRENT_SOURCE_DIR}/helper/bin2char.sh < ${CMAKE_CURRENT_BINARY_DIR}/startup.tcl > ${CMAKE_CURRENT_BINARY_DIR}/startup_tcl.inc" MAIN_DEPENDENCY ${CMAKE_CURRENT_BINARY_DIR}/startup.tcl)

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d")

add_executable(openocd openocd.c main.c hello.c ${CMAKE_CURRENT_BINARY_DIR}/startup_tcl.inc)

if (HLA_MULTICORE)
set_target_properties(openocd PROPERTIES OUTPUT_NAME "openocd_hla_multicore")
endif()

target_link_libraries(openocd 
	xsvf
	svf
	jtag
	ocdjtagdrivers
	ocdjlink
	pld
	rtt
	jim
	transport
	server
	rtos
	flash
	ocdflashnand
	ocdflashnor
	ocdhla
	ocdusbblaster
	openrisc
	target
	ws2_32 
	setupapi
	helper)
	
SET_TARGET_PROPERTIES(openocd PROPERTIES COMPILE_FLAGS "-DPKGBLDDATE=\\\"${BUILD_TIMESTAMP}\\\" -DRELSTR=\\\"\\\" -DGITVERSION=\\\"\\\" -DLIBUSB1_COMMIT=\\\"${LIBUSB1_COMMIT_HASH}\\\"")

install(TARGETS openocd DESTINATION bin)
INSTALL(FILES jtag/drivers/OpenULINK/ulink_firmware.hex DESTINATION share/openocd/OpenULINK)