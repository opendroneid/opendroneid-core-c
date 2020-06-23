project(opendroneid-core C)
set(VERSION 0.2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FORTIFY_SOURCE=2 -fstack-protector -fno-delete-null-pointer-checks -fwrapv -O0 -Wall")

cmake_minimum_required(VERSION 2.6.3)

option(BUILD_MAVLINK "Build with mavlink support" ON)
option(BUILD_WIFI "Build with WiFi support" ON)

add_subdirectory(libopendroneid)
if(BUILD_MAVLINK)
	add_subdirectory(libmav2odid)
endif()
add_subdirectory(test)
if(BUILD_WIFI)
	add_subdirectory(wifi)
endif()
