#!/bin/bash
# Build script for creating iOS XCFramework for OpenDroneID Core C Library
#
# This script builds the OpenDroneID library for iOS devices and simulators,
# then packages them into an XCFramework that can be used in iOS projects.

set -e

# Configuration
FRAMEWORK_NAME="OpenDroneID"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build_ios"
FRAMEWORK_DIR="${BUILD_DIR}/frameworks"
OUTPUT_DIR="${PROJECT_DIR}/output"

# Source files
SOURCE_FILES="${PROJECT_DIR}/libopendroneid/opendroneid.c ${PROJECT_DIR}/libopendroneid/wifi.c"
HEADER_FILES="${PROJECT_DIR}/libopendroneid/opendroneid.h ${PROJECT_DIR}/libopendroneid/odid_wifi.h"

# Compiler flags
COMMON_FLAGS="-D_FORTIFY_SOURCE=2 -fstack-protector -fno-delete-null-pointer-checks -fwrapv -O2 -Wall -Wdouble-promotion -Wno-address-of-packed-member -Wextra"

echo "================================================"
echo "Building ${FRAMEWORK_NAME} XCFramework for iOS"
echo "================================================"

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf "${BUILD_DIR}"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${BUILD_DIR}"
mkdir -p "${FRAMEWORK_DIR}"
mkdir -p "${OUTPUT_DIR}"

# Function to build for a specific platform
build_for_platform() {
    local PLATFORM=$1
    local ARCH=$2
    local SDK=$3
    local OUTPUT_LIB="${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_${PLATFORM}_${ARCH}.a"
    
    echo ""
    echo "Building for ${PLATFORM} (${ARCH})..."
    
    # Get SDK path
    local SDK_PATH=$(xcrun --sdk ${SDK} --show-sdk-path)
    
    # Compile opendroneid.c
    clang \
        -arch ${ARCH} \
        -isysroot ${SDK_PATH} \
        -mios-version-min=12.0 \
        ${COMMON_FLAGS} \
        -c ${PROJECT_DIR}/libopendroneid/opendroneid.c \
        -o "${BUILD_DIR}/${PLATFORM}_${ARCH}_opendroneid.o" \
        -I"${PROJECT_DIR}/libopendroneid"
    
    # Compile wifi.c
    clang \
        -arch ${ARCH} \
        -isysroot ${SDK_PATH} \
        -mios-version-min=12.0 \
        ${COMMON_FLAGS} \
        -c ${PROJECT_DIR}/libopendroneid/wifi.c \
        -o "${BUILD_DIR}/${PLATFORM}_${ARCH}_wifi.o" \
        -I"${PROJECT_DIR}/libopendroneid"
    
    # Create static library
    ar rcs ${OUTPUT_LIB} \
        "${BUILD_DIR}/${PLATFORM}_${ARCH}_opendroneid.o" \
        "${BUILD_DIR}/${PLATFORM}_${ARCH}_wifi.o"
    
    echo "Created library: ${OUTPUT_LIB}"
}

# Build for iOS Device (arm64)
build_for_platform "iphoneos" "arm64" "iphoneos"

# Build for iOS Simulator (arm64 and x86_64)
build_for_platform "iphonesimulator" "arm64" "iphonesimulator"
build_for_platform "iphonesimulator" "x86_64" "iphonesimulator"

# Create fat library for simulator (combine arm64 and x86_64)
echo ""
echo "Creating fat library for iOS Simulator..."
lipo -create \
    "${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_iphonesimulator_arm64.a" \
    "${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_iphonesimulator_x86_64.a" \
    -output "${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_iphonesimulator.a"

# Function to create framework structure
create_framework() {
    local PLATFORM=$1
    local LIB_FILE=$2
    local FRAMEWORK_PATH="${FRAMEWORK_DIR}/${FRAMEWORK_NAME}_${PLATFORM}.framework"
    
    echo ""
    echo "Creating framework for ${PLATFORM}..."
    
    mkdir -p "${FRAMEWORK_PATH}/Headers"
    mkdir -p "${FRAMEWORK_PATH}/Modules"
    
    # Copy library
    cp "${LIB_FILE}" "${FRAMEWORK_PATH}/${FRAMEWORK_NAME}"
    
    # Copy headers
    cp ${HEADER_FILES} "${FRAMEWORK_PATH}/Headers/"
    
    # Create module map
    cat > "${FRAMEWORK_PATH}/Modules/module.modulemap" << EOF
framework module ${FRAMEWORK_NAME} {
    umbrella header "${FRAMEWORK_NAME}.h"
    export *
    module * { export * }
}
EOF
    
    # Create umbrella header
    cat > "${FRAMEWORK_PATH}/Headers/${FRAMEWORK_NAME}.h" << EOF
//
//  ${FRAMEWORK_NAME}.h
//  ${FRAMEWORK_NAME}
//
//  Open Drone ID Core C Library
//

#ifndef ${FRAMEWORK_NAME}_h
#define ${FRAMEWORK_NAME}_h

#include "opendroneid.h"
#include "odid_wifi.h"

#endif /* ${FRAMEWORK_NAME}_h */
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
    <key>CFBundleSupportedPlatforms</key>
    <array>
        <string>${PLATFORM}</string>
    </array>
    <key>MinimumOSVersion</key>
    <string>12.0</string>
</dict>
</plist>
EOF
    
    echo "Created framework: ${FRAMEWORK_PATH}"
}

# Create frameworks for each platform
create_framework "iPhoneOS" "${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_iphoneos_arm64.a"
create_framework "iPhoneSimulator" "${FRAMEWORK_DIR}/lib${FRAMEWORK_NAME}_iphonesimulator.a"

# Create XCFramework
echo ""
echo "Creating XCFramework..."
xcodebuild -create-xcframework \
    -framework "${FRAMEWORK_DIR}/${FRAMEWORK_NAME}_iPhoneOS.framework" \
    -framework "${FRAMEWORK_DIR}/${FRAMEWORK_NAME}_iPhoneSimulator.framework" \
    -output "${OUTPUT_DIR}/${FRAMEWORK_NAME}.xcframework"

echo ""
echo "================================================"
echo "✅ Successfully created XCFramework!"
echo "================================================"
echo "Output location: ${OUTPUT_DIR}/${FRAMEWORK_NAME}.xcframework"
echo ""
echo "To use in your iOS project:"
echo "1. Drag ${FRAMEWORK_NAME}.xcframework into your Xcode project"
echo "2. In your Swift/Objective-C code, import the framework:"
echo "   Swift: import ${FRAMEWORK_NAME}"
echo "   Objective-C: #import <${FRAMEWORK_NAME}/${FRAMEWORK_NAME}.h>"
echo ""
echo "Archive size:"
du -sh "${OUTPUT_DIR}/${FRAMEWORK_NAME}.xcframework"
