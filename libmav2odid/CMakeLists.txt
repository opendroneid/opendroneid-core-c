include_directories(../libopendroneid ../mavlink_c_library_v2)

add_library(mav2odid SHARED mav2odid.c ../libopendroneid/opendroneid.c)

configure_file(libmav2odid.pc.cmake libmav2odid.pc @ONLY)

