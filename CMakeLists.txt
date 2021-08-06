project(opendroneid-core C)
set(VERSION 0.2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FORTIFY_SOURCE=2 -fstack-protector \
    -fno-delete-null-pointer-checks -fwrapv -O0 -Wall -Wdouble-promotion \
    -Wno-address-of-packed-member")

cmake_minimum_required(VERSION 2.6.3)

option(BUILD_MAVLINK "Build with mavlink support" ON)
option(BUILD_WIFI "Build with WiFi support" ON)

if(DEFINED ODID_AUTH_MAX_PAGES)
	message(STATUS "Using externally defined ODID_AUTH_MAX_PAGES value")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DODID_AUTH_MAX_PAGES=${ODID_AUTH_MAX_PAGES}")
endif()

add_subdirectory(libopendroneid)
if(BUILD_MAVLINK)
	add_subdirectory(libmav2odid)
endif()
add_subdirectory(test)
if(BUILD_WIFI)
	add_subdirectory(wifi)
endif()
