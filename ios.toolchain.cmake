# CMake toolchain file for iOS
# This file can be used with CMake to build the OpenDroneID library for iOS
#
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=ios.toolchain.cmake -DPLATFORM=OS64 ..

# Standard settings
set(CMAKE_SYSTEM_NAME iOS)
set(CMAKE_SYSTEM_VERSION 12.0)

# Define the platform (can be set via -DPLATFORM=...)
# OS64 - for iOS devices (arm64)
# SIMULATOR64 - for iOS Simulator (x86_64)
# SIMULATORARM64 - for iOS Simulator (arm64, Apple Silicon Macs)
if(NOT DEFINED PLATFORM)
    set(PLATFORM "OS64")
endif()

message(STATUS "Building for iOS platform: ${PLATFORM}")

# Set architectures based on platform
if(PLATFORM STREQUAL "OS64")
    set(CMAKE_OSX_SYSROOT iphoneos)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_SYSTEM_PROCESSOR arm64)
elseif(PLATFORM STREQUAL "SIMULATOR64")
    set(CMAKE_OSX_SYSROOT iphonesimulator)
    set(CMAKE_OSX_ARCHITECTURES x86_64)
    set(CMAKE_SYSTEM_PROCESSOR x86_64)
elseif(PLATFORM STREQUAL "SIMULATORARM64")
    set(CMAKE_OSX_SYSROOT iphonesimulator)
    set(CMAKE_OSX_ARCHITECTURES arm64)
    set(CMAKE_SYSTEM_PROCESSOR arm64)
else()
    message(FATAL_ERROR "Invalid PLATFORM: ${PLATFORM}")
endif()

# Deployment target
set(CMAKE_OSX_DEPLOYMENT_TARGET "12.0" CACHE STRING "Minimum iOS version")

# Compiler settings
set(CMAKE_C_FLAGS_INIT "-fno-common")
set(CMAKE_CXX_FLAGS_INIT "-fno-common")

# Skip compiler checks (they can fail for cross-compilation)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

# Note: Bitcode is no longer required or recommended for iOS as of Xcode 14
# It has been deprecated by Apple and removed from the toolchain

# Search paths
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

message(STATUS "iOS Toolchain configured")
message(STATUS "  Platform: ${PLATFORM}")
message(STATUS "  SDK: ${CMAKE_OSX_SYSROOT}")
message(STATUS "  Architecture: ${CMAKE_OSX_ARCHITECTURES}")
message(STATUS "  Deployment Target: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
