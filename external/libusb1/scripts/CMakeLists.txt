CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

set(LIBUSB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../source/libusb)
CONFIGURE_FILE(${LIBUSB_SOURCE_DIR}/libusb.h ${CMAKE_CURRENT_BINARY_DIR}/public/libusb.h)

execute_process(COMMAND git rev-parse HEAD
		WORKING_DIRECTORY ${LIBUSB_SOURCE_DIR}
        OUTPUT_VARIABLE DETECTED_LIBUSB1_COMMIT_HASH
		OUTPUT_STRIP_TRAILING_WHITESPACE) 

set(LIBUSB1_COMMIT_HASH ${DETECTED_LIBUSB1_COMMIT_HASH} PARENT_SCOPE)

add_library(libusb1 STATIC
	${LIBUSB_SOURCE_DIR}/core.c
	${LIBUSB_SOURCE_DIR}/descriptor.c
	${LIBUSB_SOURCE_DIR}/hotplug.c
	${LIBUSB_SOURCE_DIR}/io.c
	${LIBUSB_SOURCE_DIR}/strerror.c
	${LIBUSB_SOURCE_DIR}/sync.c
	${LIBUSB_SOURCE_DIR}/os/poll_windows.c
	${LIBUSB_SOURCE_DIR}/os/windows_nt_common.c
	${LIBUSB_SOURCE_DIR}/os/windows_usbdk.c
	${LIBUSB_SOURCE_DIR}/os/windows_winusb.c
	${LIBUSB_SOURCE_DIR}/os/threads_windows.c)

target_compile_definitions(libusb1 PRIVATE HAVE_CONFIG_H)
target_include_directories(libusb1 INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/public)
target_include_directories(libusb1 PRIVATE ${LIBUSB_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
