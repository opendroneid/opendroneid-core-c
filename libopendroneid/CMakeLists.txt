add_library(opendroneid SHARED opendroneid.c wifi.c)

configure_file(libopendroneid.pc.cmake libopendroneid.pc @ONLY)

