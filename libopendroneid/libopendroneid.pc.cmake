prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@BIN_INSTALL_DIR@
libdir=@LIB_INSTALL_DIR@
includedir=@INCLUDE_INSTALL_DIR@

Name: libopendroneid
Version: @VERSION@
Description: OpenDroneID reference library
Requires.private: 
Libs.private: -lm
Libs: -L${libdir} -lopendroneid
Cflags: -I${includedir}
