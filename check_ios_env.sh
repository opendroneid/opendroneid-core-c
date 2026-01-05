#!/bin/bash
# Check if the build environment is ready for iOS framework compilation
# 检查构建环境是否准备好进行 iOS 框架编译

set -e

echo "================================================"
echo "iOS Framework Build Environment Check"
echo "iOS 框架构建环境检查"
echo "================================================"
echo ""

# Check if running on macOS
echo "1. Checking operating system..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "   ✅ macOS detected"
else
    echo "   ❌ Not running on macOS"
    echo "   iOS framework can only be built on macOS"
    echo "   iOS 框架只能在 macOS 上构建"
    exit 1
fi
echo ""

# Check for Xcode
echo "2. Checking for Xcode..."
if command -v xcodebuild &> /dev/null; then
    XCODE_VERSION=$(xcodebuild -version | head -n 1)
    echo "   ✅ $XCODE_VERSION"
else
    echo "   ❌ Xcode not found"
    echo "   Please install Xcode from the App Store"
    echo "   请从 App Store 安装 Xcode"
    exit 1
fi
echo ""

# Check for Xcode Command Line Tools
echo "3. Checking for Xcode Command Line Tools..."
if xcode-select -p &> /dev/null; then
    XCODE_PATH=$(xcode-select -p)
    echo "   ✅ Command Line Tools installed at: $XCODE_PATH"
else
    echo "   ❌ Xcode Command Line Tools not found"
    echo "   Please run: xcode-select --install"
    echo "   请运行: xcode-select --install"
    exit 1
fi
echo ""

# Check for iOS SDK
echo "4. Checking for iOS SDK..."
if xcodebuild -showsdks 2>/dev/null | grep -q "iphoneos"; then
    IOS_SDK=$(xcodebuild -showsdks 2>/dev/null | grep "iphoneos" | tail -n 1 | awk '{print $NF}')
    echo "   ✅ iOS SDK found: $IOS_SDK"
else
    echo "   ❌ iOS SDK not found"
    echo "   Please ensure Xcode is properly installed"
    echo "   请确保 Xcode 已正确安装"
    exit 1
fi
echo ""

# Check for iOS Simulator SDK
echo "5. Checking for iOS Simulator SDK..."
if xcodebuild -showsdks 2>/dev/null | grep -q "iphonesimulator"; then
    SIM_SDK=$(xcodebuild -showsdks 2>/dev/null | grep "iphonesimulator" | tail -n 1 | awk '{print $NF}')
    echo "   ✅ iOS Simulator SDK found: $SIM_SDK"
else
    echo "   ⚠️  iOS Simulator SDK not found"
    echo "   The framework can still be built for device only"
    echo "   仍可为设备构建框架"
fi
echo ""

# Check available architectures
echo "6. Checking available architectures..."
echo "   Device architectures:"
DEVICE_ARCH=$(xcodebuild -showsdks 2>/dev/null | grep iphoneos | tail -n1)
echo "     • arm64 (required)"

echo "   Simulator architectures:"
HOST_ARCH=$(uname -m)
if [[ "$HOST_ARCH" == "arm64" ]]; then
    echo "     • arm64 (Apple Silicon Mac detected)"
    echo "     • x86_64 (Rosetta 2 for Intel Macs)"
else
    echo "     • x86_64 (Intel Mac detected)"
    echo "     • arm64 (for Apple Silicon Macs)"
fi
echo ""

# Check for required tools
echo "7. Checking for required build tools..."

if command -v clang &> /dev/null; then
    echo "   ✅ clang compiler found"
else
    echo "   ❌ clang not found"
    exit 1
fi

if command -v ar &> /dev/null; then
    echo "   ✅ ar archiver found"
else
    echo "   ❌ ar not found"
    exit 1
fi

if command -v lipo &> /dev/null; then
    echo "   ✅ lipo tool found"
else
    echo "   ❌ lipo not found"
    exit 1
fi
echo ""

# Optional: Check for CMake
echo "8. Checking for optional tools..."
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n 1)
    echo "   ✅ $CMAKE_VERSION (for build_ios_cmake.sh)"
else
    echo "   ℹ️  CMake not found (optional, only needed for build_ios_cmake.sh)"
    echo "      You can install it with: brew install cmake"
    echo "      你可以使用以下命令安装: brew install cmake"
fi
echo ""

# Summary
echo "================================================"
echo "Environment Check Complete!"
echo "环境检查完成!"
echo "================================================"
echo ""
echo "✅ Your system is ready to build the iOS framework"
echo "✅ 你的系统已准备好构建 iOS 框架"
echo ""
echo "To build the framework, run:"
echo "要构建框架，请运行:"
echo "   ./build_ios_framework.sh"
echo ""
echo "For more information, see:"
echo "获取更多信息，请参阅:"
echo "   - README_iOS.md (English)"
echo "   - QUICKSTART_iOS_CN.md (中文)"
echo ""
