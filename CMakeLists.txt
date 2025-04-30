cmake_minimum_required(VERSION 3.5)

project(opendroneid-core C)
set(VERSION 0.2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FORTIFY_SOURCE=2 -fstack-protector \
    -fno-delete-null-pointer-checks -fwrapv -O0 -Wall -Wdouble-promotion \
    -Wno-address-of-packed-member -Wextra")

option(BUILD_MAVLINK "Build with mavlink support" ON)
option(BUILD_WIFI "Build with WiFi support" ON)
option(BUILD_TESTS "Build unit/debug tests" ON)

if(DEFINED ODID_AUTH_MAX_PAGES)
	message(STATUS "Using externally defined ODID_AUTH_MAX_PAGES value")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DODID_AUTH_MAX_PAGES=${ODID_AUTH_MAX_PAGES}")
endif()

if(DEFINED ODID_BASIC_ID_MAX_MESSAGES)
	message(STATUS "Using externally defined ODID_BASIC_ID_MAX_MESSAGES value")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DODID_BASIC_ID_MAX_MESSAGES=${ODID_BASIC_ID_MAX_MESSAGES}")
endif()

add_subdirectory(libopendroneid)
if(BUILD_MAVLINK)
	add_subdirectory(libmav2odid)
endif()
if(BUILD_TESTS)
	enable_testing()
	add_subdirectory(test)
endif()
if(BUILD_WIFI)
	add_subdirectory(wifi)
endif()
