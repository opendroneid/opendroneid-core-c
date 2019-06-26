project(opendroneid-core C)
set(VERSION 0.1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_FORTIFY_SOURCE=2 -fstack-protector -fno-delete-null-pointer-checks -fwrapv -O0")

cmake_minimum_required(VERSION 2.6.3)

add_subdirectory(libopendroneid)
add_subdirectory(test)
