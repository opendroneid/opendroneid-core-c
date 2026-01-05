#!/bin/bash
# Alternative CMake-based build script for iOS
# This script uses CMake with the iOS toolchain to build static libraries
# Then manually creates the framework structure

set -e

FRAMEWORK_NAME="OpenDroneID"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build_ios_cmake"
OUTPUT_DIR="${PROJECT_DIR}/output"

echo "================================================"
echo "Building ${FRAMEWORK_NAME} for iOS using CMake"
echo "================================================"

# Check for CMake
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake is not installed"
    echo "Please install CMake: brew install cmake"
    exit 1
fi

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf "${BUILD_DIR}"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${BUILD_DIR}"
mkdir -p "${OUTPUT_DIR}"

# Function to build for a specific platform
build_cmake_platform() {
    local PLATFORM=$1
    local PLATFORM_NAME=$2
    local BUILD_SUBDIR="${BUILD_DIR}/${PLATFORM_NAME}"
    
    echo ""
    echo "Building for ${PLATFORM_NAME} using CMake..."
    
    mkdir -p "${BUILD_SUBDIR}"
    cd "${BUILD_SUBDIR}"
    
    # Configure
    cmake "${PROJECT_DIR}" \
        -DCMAKE_TOOLCHAIN_FILE="${PROJECT_DIR}/ios.toolchain.cmake" \
        -DPLATFORM=${PLATFORM} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${BUILD_SUBDIR}/install" \
        -DBUILD_WIFI=OFF \
        -DBUILD_MAVLINK=OFF \
        -DBUILD_TESTS=OFF
    
    # Build
    cmake --build . --config Release
    
    echo "Built library for ${PLATFORM_NAME}"
}

# Build for each platform
build_cmake_platform "OS64" "iphoneos"
build_cmake_platform "SIMULATORARM64" "iphonesimulator-arm64"
build_cmake_platform "SIMULATOR64" "iphonesimulator-x86_64"

# Create fat library for simulator
echo ""
echo "Creating fat library for iOS Simulator..."
lipo -create \
    "${BUILD_DIR}/iphonesimulator-arm64/libopendroneid/libopendroneid.a" \
    "${BUILD_DIR}/iphonesimulator-x86_64/libopendroneid/libopendroneid.a" \
    -output "${BUILD_DIR}/libopendroneid_simulator.a"

# Create framework structures
create_framework() {
    local PLATFORM=$1
    local LIB_PATH=$2
    local FRAMEWORK_PATH="${BUILD_DIR}/${FRAMEWORK_NAME}_${PLATFORM}.framework"
    
    echo ""
    echo "Creating framework for ${PLATFORM}..."
    
    mkdir -p "${FRAMEWORK_PATH}/Headers"
    mkdir -p "${FRAMEWORK_PATH}/Modules"
    
    # Copy library
    cp "${LIB_PATH}" "${FRAMEWORK_PATH}/${FRAMEWORK_NAME}"
    
    # Copy headers
    cp "${PROJECT_DIR}/libopendroneid/opendroneid.h" "${FRAMEWORK_PATH}/Headers/"
    cp "${PROJECT_DIR}/libopendroneid/odid_wifi.h" "${FRAMEWORK_PATH}/Headers/"
    
    # Create umbrella header
    cat > "${FRAMEWORK_PATH}/Headers/${FRAMEWORK_NAME}.h" << EOF
//
//  ${FRAMEWORK_NAME}.h
//  ${FRAMEWORK_NAME} Framework
//
//  Open Drone ID Core C Library
//

#ifndef ${FRAMEWORK_NAME}_h
#define ${FRAMEWORK_NAME}_h

#include "opendroneid.h"
#include "odid_wifi.h"

#endif /* ${FRAMEWORK_NAME}_h */
EOF
    
    # Create module map
    cat > "${FRAMEWORK_PATH}/Modules/module.modulemap" << EOF
framework module ${FRAMEWORK_NAME} {
    umbrella header "${FRAMEWORK_NAME}.h"
    export *
    module * { export * }
}
EOF
    
    # Create Info.plist
    cat > "${FRAMEWORK_PATH}/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleDevelopmentRegion</key>
    <string>en</string>
    <key>CFBundleExecutable</key>
    <string>${FRAMEWORK_NAME}</string>
    <key>CFBundleIdentifier</key>
    <string>org.opendroneid.${FRAMEWORK_NAME}</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleName</key>
    <string>${FRAMEWORK_NAME}</string>
    <key>CFBundlePackageType</key>
    <string>FMWK</string>
    <key>CFBundleShortVersionString</key>
    <string>0.2.0</string>
    <key>CFBundleVersion</key>
    <string>1</string>
    <key>MinimumOSVersion</key>
    <string>12.0</string>
</dict>
</plist>
EOF
    
    echo "Created framework: ${FRAMEWORK_PATH}"
}

# Create frameworks
create_framework "iPhoneOS" "${BUILD_DIR}/iphoneos/libopendroneid/libopendroneid.a"
create_framework "iPhoneSimulator" "${BUILD_DIR}/libopendroneid_simulator.a"

# Create XCFramework
echo ""
echo "Creating XCFramework..."
xcodebuild -create-xcframework \
    -framework "${BUILD_DIR}/${FRAMEWORK_NAME}_iPhoneOS.framework" \
    -framework "${BUILD_DIR}/${FRAMEWORK_NAME}_iPhoneSimulator.framework" \
    -output "${OUTPUT_DIR}/${FRAMEWORK_NAME}.xcframework"

echo ""
echo "================================================"
echo "✅ Successfully created XCFramework!"
echo "================================================"
echo "Output: ${OUTPUT_DIR}/${FRAMEWORK_NAME}.xcframework"

cd "${PROJECT_DIR}"
