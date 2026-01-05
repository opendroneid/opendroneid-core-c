# OpenDroneID iOS 框架 - 完整交付说明

## 概述

我已经为你创建了完整的 iOS 框架构建系统，用于将 Open Drone ID Core C Library 打包成 iOS 可用的 framework。这个框架可以用来解析无人机通过蓝牙或 Wi-Fi 传输的数据。

## 已创建的文件

### 1. 构建脚本 (Build Scripts)

#### `build_ios_framework.sh` (主要构建脚本)
- 使用 clang 编译器直接构建
- 支持 iOS 设备 (arm64)
- 支持 iOS 模拟器 (arm64 + x86_64)
- 自动创建 XCFramework 格式的框架
- **使用方法**: 在 macOS 上运行 `./build_ios_framework.sh`

#### `build_ios_cmake.sh` (CMake 构建脚本)
- 使用 CMake 构建系统
- 功能与主脚本相同
- 适合熟悉 CMake 的开发者

#### `check_ios_env.sh` (环境检查脚本)
- 检查是否在 macOS 上运行
- 验证 Xcode 是否已安装
- 检查所有必需的工具
- **使用方法**: 运行 `./check_ios_env.sh`

#### `ios.toolchain.cmake` (CMake 工具链文件)
- CMake iOS 交叉编译配置
- 用于 CMake 构建方式

### 2. 文档 (Documentation)

#### `README_iOS.md` (英文完整文档)
包含内容：
- 详细的构建说明
- 集成到 Xcode 项目的步骤
- Swift 和 Objective-C 代码示例
- 蓝牙扫描完整示例
- 所有消息类型的说明
- 故障排除指南

#### `QUICKSTART_iOS_CN.md` (中文快速开始指南)
包含内容：
- 简明的构建步骤
- Swift 代码示例
- 蓝牙扫描示例
- 常见问题解决

#### `iOS_FRAMEWORK_VALIDATION.md` (验证文档)
- 总结所有交付内容
- 框架特性说明
- 使用流程说明

### 3. 示例代码 (Example Code)

#### `examples/ios/DroneIDParser.swift` (完整 Swift 实现)
这是一个完整的 Swift 实现，包含：

**解析器类 (DroneIDParser)**
- `parseBasicID()` - 解析基础 ID 信息
- `parseLocation()` - 解析位置信息
- `parseSystem()` - 解析系统信息
- `parseSelfID()` - 解析自我描述
- `parseOperatorID()` - 解析操作员 ID
- `parseMessagePack()` - 解析消息包

**数据模型**
- `BasicIDInfo` - 基础 ID 信息结构
- `LocationInfo` - 位置信息结构
- `SystemInfo` - 系统信息结构
- `UASData` - 完整的无人机数据

**蓝牙扫描器 (DroneIDBluetoothScanner)**
- 自动扫描附近的无人机
- 解析蓝牙广播数据
- 回调函数通知检测到的无人机

## 如何使用

### 第一步：构建框架

1. 确保你有 macOS 电脑并安装了 Xcode

2. 打开终端，进入项目目录：
```bash
cd opendroneid-core-c
```

3. (可选) 检查环境：
```bash
./check_ios_env.sh
```

4. 构建框架：
```bash
./build_ios_framework.sh
```

5. 构建完成后，你会得到：
```
output/OpenDroneID.xcframework
```

### 第二步：集成到你的 iOS 项目

1. 在 Xcode 中打开你的项目

2. 将 `OpenDroneID.xcframework` 拖入项目

3. 在项目设置中，确保框架设置为 "Embed & Sign"

4. 在 `Info.plist` 添加蓝牙权限：
```xml
<key>NSBluetoothAlwaysUsageDescription</key>
<string>需要蓝牙权限来检测附近的无人机</string>
```

### 第三步：在代码中使用

#### 基本使用示例

```swift
import OpenDroneID

// 创建解析器
let parser = DroneIDParser()

// 解析无人机数据
func handleDroneData(_ data: Data) {
    // 解析基础 ID
    if let basicID = parser.parseBasicID(data: data) {
        print("无人机 ID: \(basicID.uasID)")
        print("无人机类型: \(basicID.uaType)")
    }
    
    // 解析位置信息
    if let location = parser.parseLocation(data: data) {
        print("经度: \(location.latitude)")
        print("纬度: \(location.longitude)")
        print("高度: \(location.altitudeBarometric) 米")
        print("速度: \(location.horizontalSpeed) 米/秒")
    }
}
```

#### 完整的蓝牙扫描示例

```swift
import UIKit

class DroneViewController: UIViewController {
    let scanner = DroneIDBluetoothScanner()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 设置回调函数
        scanner.onDroneDetected = { [weak self] uasData in
            // 处理检测到的无人机数据
            if let basicID = uasData.basicID.first {
                print("检测到无人机: \(basicID.uasID)")
            }
            
            if let location = uasData.location {
                print("位置: \(location.latitude), \(location.longitude)")
                print("高度: \(location.altitudeBarometric) 米")
                
                // 更新 UI
                DispatchQueue.main.async {
                    self?.updateUI(with: location)
                }
            }
        }
    }
    
    @IBAction func startScanning(_ sender: Any) {
        scanner.startScanning()
    }
    
    @IBAction func stopScanning(_ sender: Any) {
        scanner.stopScanning()
    }
    
    func updateUI(with location: LocationInfo) {
        // 在这里更新你的界面，比如在地图上显示无人机位置
    }
}
```

## 支持的消息类型

框架支持解析以下所有类型的无人机消息：

1. **Basic ID (基础 ID)** 
   - 无人机序列号或会话 ID
   - 无人机类型（多旋翼、固定翼等）

2. **Location (位置信息)**
   - 实时经纬度坐标
   - 气压高度、大地高度
   - 水平和垂直速度
   - 飞行方向
   - 时间戳

3. **System (系统信息)**
   - 操作员位置
   - 飞行区域信息
   - 区域半径、上限和下限

4. **Self ID (自我描述)**
   - 操作员设置的文本描述

5. **Operator ID (操作员 ID)**
   - 操作员注册 ID

6. **Authentication (认证)**
   - 认证数据和签名

7. **Message Pack (消息包)**
   - 包含多种消息类型的组合

## 技术规格

### 支持的架构
- iOS 设备: arm64
- iOS 模拟器: arm64 (Apple Silicon Mac)
- iOS 模拟器: x86_64 (Intel Mac)

### 最低系统要求
- iOS 12.0 或更高版本

### 标准合规性
- ASTM F3411-22a (Remote ID and Tracking)
- ASD-STAN prEN 4709-002 (Direct Remote Identification)
- 协议版本: 2

## 常见问题

### Q: 在哪里可以获取无人机数据？
A: 无人机数据通过蓝牙广播发送。使用 CoreBluetooth 框架扫描蓝牙设备，从广播数据中提取无人机消息，然后使用这个框架解析。

### Q: 需要什么样的蓝牙权限？
A: 需要在 Info.plist 中添加蓝牙使用说明。详见文档中的权限配置部分。

### Q: iOS 支持哪些传输方式？
A: iOS 目前主要支持 Bluetooth 4.0 传统广播。Wi-Fi NaN 和 Beacon 接收在 iOS 上受到限制。

### Q: 如何测试框架？
A: 你可以：
1. 使用 ESP32 等硬件构建无人机发射器
2. 使用 Android 接收器应用验证发射
3. 使用你的 iOS 应用接收和解析数据

### Q: 框架大小是多少？
A: 编译后的框架大小约为几百 KB，非常轻量。

## 文件结构

```
opendroneid-core-c/
├── build_ios_framework.sh          # 主构建脚本
├── build_ios_cmake.sh              # CMake 构建脚本
├── check_ios_env.sh                # 环境检查脚本
├── ios.toolchain.cmake             # CMake 工具链
├── README_iOS.md                   # 英文完整文档
├── QUICKSTART_iOS_CN.md            # 中文快速指南
├── iOS_FRAMEWORK_VALIDATION.md     # 验证文档
├── examples/ios/
│   ├── DroneIDParser.swift         # 完整 Swift 示例
│   └── README.md                   # 示例说明
└── output/                         # 构建输出目录
    └── OpenDroneID.xcframework     # 生成的框架
```

## 获取帮助

如果遇到问题：

1. 查看 `README_iOS.md` 获取详细文档
2. 查看 `QUICKSTART_iOS_CN.md` 获取快速指南
3. 运行 `./check_ios_env.sh` 检查环境配置
4. 查看示例代码 `examples/ios/DroneIDParser.swift`
5. 在 GitHub 上提出 issue

## 重要提示

1. **必须在 macOS 上构建**: iOS 框架只能在 macOS 系统上使用 Xcode 构建

2. **需要实际硬件测试**: 无人机信号检测需要在真实的 iOS 设备上测试，模拟器无法接收蓝牙信号

3. **蓝牙权限**: 确保正确配置蓝牙权限，否则无法扫描

4. **标准合规**: 框架符合最新的 ASTM 和 ASD-STAN 标准

## 许可证

Apache-2.0 (与主库相同)

## 开始使用

现在你可以：

1. 运行 `./check_ios_env.sh` 检查环境
2. 运行 `./build_ios_framework.sh` 构建框架
3. 将框架集成到你的 iOS 项目
4. 使用示例代码开始解析无人机数据

祝你开发顺利！如有问题，请参考文档或在 GitHub 上提出 issue。

---

创建日期: 2026-01-05
版本: 0.2.0
