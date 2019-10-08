SET(CMAKE_SYSTEM_NAME Windows)
SET(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/gcc.exe")
SET(CMAKE_CXX_COMPILER "${TOOLCHAIN_ROOT}/bin/g++.exe")

if(EXISTS "$ENV{TOOLCHAIN_ROOT}/Qt/v5-CMake/Qt5Cross.cmake")
	include("$ENV{TOOLCHAIN_ROOT}/Qt/v5-CMake/Qt5Cross.cmake")
endif()
