add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/jtag/minidriver_imp.h COMMAND cp ARGS ${CMAKE_CURRENT_SOURCE_DIR}/drivers/minidriver_imp.h ${CMAKE_CURRENT_BINARY_DIR}/jtag/minidriver_imp.h  MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/drivers/minidriver_imp.h)

include_directories(drivers drivers/libjaylink)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/drivers/minidriver_imp.h ${CMAKE_CURRENT_BINARY_DIR}/minidriver_imp.h)

SET(JAYLINK_VERSION_PACKAGE_MAJOR 0)
SET(JAYLINK_VERSION_PACKAGE_MINOR 1)
SET(JAYLINK_VERSION_PACKAGE_MICRO 0)
SET(JAYLINK_VERSION_PACKAGE "${JAYLINK_VERSION_PACKAGE_MAJOR}.${JAYLINK_VERSION_PACKAGE_MINOR}.${JAYLINK_VERSION_PACKAGE_MICRO}")

set(JAYLINK_VERSION_LIB_CURRENT 0)
set(JAYLINK_VERSION_LIB_REVISION 0)
set(JAYLINK_VERSION_LIB_AGE 0)
set(JAYLINK_VERSION_LIB "{$JAYLINK_VERSION_LIB_CURRENT}:{$JAYLINK_VERSION_LIB_REVISION}:{$JAYLINK_VERSION_LIB_AGE}")

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/drivers/libjaylink/libjaylink/version.h.in ${CMAKE_CURRENT_BINARY_DIR}/jlink/version.h)
add_library(ocdhla STATIC hla/hla_transport.c hla/hla_tcl.c hla/hla_interface.c hla/hla_layout.c)
add_library(ocdaice STATIC aice/aice_transport.c aice/aice_interface.c aice/aice_port.c aice/aice_usb.c aice/aice_pipe.c)
add_library(ocdusbblaster STATIC drivers/usb_blaster/usb_blaster.c drivers/usb_blaster/ublast2_access_libusb.c)

add_library(ocdjtagdrivers STATIC
	drivers/driver.c
	drivers/libusb1_common.c
	drivers/usb_common.c
	drivers/ftdi.c
	drivers/mpsse.c
	drivers/usbprog.c
	drivers/rlink.c
	drivers/rlink_speed_table.c
	drivers/ulink.c
	drivers/versaloon/usbtoxxx/usbtogpio.c
	drivers/versaloon/usbtoxxx/usbtojtagraw.c
	drivers/versaloon/usbtoxxx/usbtoswd.c
	drivers/versaloon/usbtoxxx/usbtopwr.c
	drivers/versaloon/usbtoxxx/usbtoxxx.c
	drivers/versaloon/versaloon.c
	drivers/vsllink.c
	drivers/arm-jtag-ew.c
	drivers/stlink_usb.c
	drivers/ti_icdi_usb.c
	drivers/osbdm.c
	drivers/kitprog.c
	drivers/opendous.c
	drivers/cmsis_dap_usb.c)

	
add_library(ocdjlink STATIC
	drivers/jlink.c
	drivers/libjaylink/libjaylink/buffer.c
	drivers/libjaylink/libjaylink/core.c
	drivers/libjaylink/libjaylink/device.c
	drivers/libjaylink/libjaylink/discovery.c
	drivers/libjaylink/libjaylink/emucom.c
	drivers/libjaylink/libjaylink/error.c
	drivers/libjaylink/libjaylink/fileio.c
	drivers/libjaylink/libjaylink/jtag.c
	drivers/libjaylink/libjaylink/list.c
	drivers/libjaylink/libjaylink/log.c
	drivers/libjaylink/libjaylink/strutil.c
	drivers/libjaylink/libjaylink/swd.c
	drivers/libjaylink/libjaylink/swo.c
	drivers/libjaylink/libjaylink/target.c
	drivers/libjaylink/libjaylink/transport.c
	drivers/libjaylink/libjaylink/util.c
	drivers/libjaylink/libjaylink/version.c) 
	
add_library(jtag STATIC adapter.c core.c interface.c interfaces.c tcl.c commands.c)

SET_TARGET_PROPERTIES(ocdjlink PROPERTIES COMPILE_FLAGS -I${CMAKE_CURRENT_BINARY_DIR}/jlink)