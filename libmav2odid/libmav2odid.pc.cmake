prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@BIN_INSTALL_DIR@
libdir=@LIB_INSTALL_DIR@
includedir=@INCLUDE_INSTALL_DIR@

Name: libmav2odid
Version: @VERSION@
Description: Mavlink to OpenDroneID reference library
Requires.private: 
Libs.private: -lm
Libs: -L${libdir} -lmav2odid
Cflags: -I${includedir}
