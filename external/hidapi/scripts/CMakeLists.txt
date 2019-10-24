CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

set(HIDAPI_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../source)

add_library(hidapi STATIC
	${HIDAPI_SOURCE_DIR}/windows/hid.c
)

target_compile_definitions(hidapi PRIVATE HAVE_CONFIG_H)
target_include_directories(hidapi PRIVATE ${HIDAPI_SOURCE_DIR}/hidapi)
target_include_directories(hidapi INTERFACE ${HIDAPI_SOURCE_DIR}/hidapi)
