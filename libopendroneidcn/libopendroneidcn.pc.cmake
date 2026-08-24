prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@BIN_INSTALL_DIR@
libdir=@LIB_INSTALL_DIR@
includedir=@INCLUDE_INSTALL_DIR@

Name: libopendroneidcn
Version: @VERSION@
Description: OpenDroneID GB 46750-2025 reference library
Requires.private:
Libs: -L${libdir} -lopendroneidcn
Cflags: -I${includedir}
