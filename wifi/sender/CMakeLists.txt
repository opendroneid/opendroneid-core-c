find_package(PkgConfig)

pkg_check_modules(GPS REQUIRED libgps)
pkg_check_modules(NL QUIET libnl-tiny)
if (NOT NL_FOUND)
	pkg_check_modules(NL REQUIRED libnl-genl-3.0)
endif(NOT NL_FOUND)

link_libraries(opendroneid m ${GPS_LIBRARIES} ${NL_LIBRARIES} ${GENL_LIBRARIES})
include_directories(../../libopendroneid ${GPS_INCLUDE_DIRS} ${NL_INCLUDE_DIRS} ${GENL_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GPS_CFLAGS_OTHER} ${NL_CFLAGS_OTHER} ${GENL_CFLAGS_OTHER}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -W -Wno-unused-parameter -std=gnu99 -fno-strict-aliasing -MD -MP -D_GNU_SOURCE")
if (GPS_VERSION VERSION_LESS 3.20)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DLIBGPS_OLD")
endif()

add_executable(sender main.c)

install(TARGETS sender DESTINATION bin)
