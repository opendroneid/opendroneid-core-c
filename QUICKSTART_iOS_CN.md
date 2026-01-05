# Quick Start Guide - iOS Framework

这是一个快速开始指南，帮助你在 iOS 项目中使用 OpenDroneID 框架来解析无人机数据。

## 构建框架 (Building the Framework)

### 前提条件 (Prerequisites)
- macOS 电脑
- 安装 Xcode (从 App Store 下载)
- Xcode Command Line Tools

### 构建步骤 (Build Steps)

1. 打开终端 (Terminal)

2. 克隆或下载此代码库

3. 进入项目目录：
```bash
cd opendroneid-core-c
```

4. 运行构建脚本：
```bash
./build_ios_framework.sh
```

5. 构建完成后，框架会生成在 `output/OpenDroneID.xcframework` 目录中

## 集成到你的 iOS 项目 (Integration)

### 步骤 1: 添加框架到项目

1. 在 Xcode 中打开你的 iOS 项目
2. 将 `OpenDroneID.xcframework` 拖拽到项目导航器中
3. 在弹出的对话框中，确保选中 "Copy items if needed"
4. 选择你的 target

### 步骤 2: 配置项目

在 Project Navigator 中：
1. 选择你的项目
2. 选择你的 target
3. 在 "General" 标签页中
4. 找到 "Frameworks, Libraries, and Embedded Content"
5. 确保 OpenDroneID.xcframework 的 Embed 设置为 "Embed & Sign"

### 步骤 3: 添加蓝牙权限

在你的 `Info.plist` 文件中添加以下权限说明：

```xml
<key>NSBluetoothAlwaysUsageDescription</key>
<string>需要蓝牙权限来检测附近的无人机</string>
<key>NSBluetoothPeripheralUsageDescription</key>
<string>需要蓝牙权限来检测附近的无人机</string>
```

## 使用示例 (Usage Examples)

### Swift 代码示例

```swift
import OpenDroneID

// 创建解析器
let parser = DroneIDParser()

// 解析基础 ID 信息
func parseDroneBasicID(data: Data) {
    if let basicID = parser.parseBasicID(data: data) {
        print("无人机 ID: \(basicID.uasID)")
        print("无人机类型: \(basicID.uaType)")
    }
}

// 解析位置信息
func parseDroneLocation(data: Data) {
    if let location = parser.parseLocation(data: data) {
        print("纬度: \(location.latitude)")
        print("经度: \(location.longitude)")
        print("高度: \(location.altitudeBarometric) 米")
        print("速度: \(location.horizontalSpeed) 米/秒")
        print("方向: \(location.direction) 度")
    }
}
```

### 完整的蓝牙扫描示例

```swift
import UIKit
import CoreBluetooth

class DroneViewController: UIViewController {
    let scanner = DroneIDBluetoothScanner()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // 设置检测到无人机时的回调
        scanner.onDroneDetected = { [weak self] uasData in
            self?.handleDroneDetected(uasData)
        }
    }
    
    func handleDroneDetected(_ uasData: UASData) {
        // 处理基础 ID
        if let basicID = uasData.basicID.first {
            print("检测到无人机 ID: \(basicID.uasID)")
        }
        
        // 处理位置信息
        if let location = uasData.location {
            print("无人机位置: \(location.latitude), \(location.longitude)")
            print("高度: \(location.altitudeBarometric) 米")
            print("速度: \(location.horizontalSpeed) 米/秒")
            
            // 更新 UI
            DispatchQueue.main.async {
                // 在地图上显示无人机位置
                // self.updateMapWithDrone(location)
            }
        }
        
        // 处理操作员信息
        if let system = uasData.system {
            print("操作员位置: \(system.operatorLatitude), \(system.operatorLongitude)")
        }
    }
    
    @IBAction func startScanning(_ sender: UIButton) {
        scanner.startScanning()
        print("开始扫描无人机...")
    }
    
    @IBAction func stopScanning(_ sender: UIButton) {
        scanner.stopScanning()
        print("停止扫描")
    }
}
```

## 主要功能 (Key Features)

### 支持的消息类型

1. **Basic ID (基础 ID)** - 无人机的唯一标识
   - 序列号或会话 ID
   - 无人机类型（直升机、固定翼等）

2. **Location (位置)** - 实时位置和运动数据
   - 经纬度坐标
   - 高度（气压高度、大地高度）
   - 速度和方向

3. **System (系统)** - 操作员和系统信息
   - 操作员位置
   - 飞行区域信息

4. **Self ID (自我描述)** - 操作员设置的描述文本

5. **Operator ID (操作员 ID)** - 操作员注册 ID

6. **Message Pack (消息包)** - 包含多种消息类型的容器

### 核心解码函数

```c
// 解码各种消息类型
int decodeBasicIDMessage(...)      // 解码基础 ID
int decodeLocationMessage(...)     // 解码位置信息
int decodeSystemMessage(...)       // 解码系统信息
int decodeSelfIDMessage(...)       // 解码自我描述
int decodeOperatorIDMessage(...)   // 解码操作员 ID
int decodeMessagePack(...)         // 解码消息包
```

## 数据接收方式 (Data Reception)

### 蓝牙 (Bluetooth)
- Bluetooth 4.0 传统广播
- Bluetooth 5.0 长距离广播
- iOS 支持接收 BT4 传统广播信号

### Wi-Fi
- Wi-Fi Beacon 帧
- Wi-Fi NaN (邻居感知网络)
- 注意：iOS 目前 API 限制，Wi-Fi 帧接收支持有限

## 测试 (Testing)

### 使用真实硬件测试
1. 使用 ESP32 或其他支持的硬件构建发射器
2. 使用 Android 接收器应用验证发射
3. 使用你的 iOS 应用测试接收

### 使用模拟数据测试
可以创建符合 ASTM F3411 规范的模拟数据进行测试。

## 标准合规性 (Standards Compliance)

此库符合以下标准：
- ASTM F3411-22a (Remote ID and Tracking)
- ASD-STAN prEN 4709-002 (Direct Remote Identification)
- 协议版本：2

## 故障排除 (Troubleshooting)

### 构建问题
```bash
# 安装 Xcode Command Line Tools
xcode-select --install

# 检查 iOS SDK
xcodebuild -showsdks
```

### 运行时问题
1. 确保框架正确嵌入到应用中
2. 检查 iOS 部署目标至少为 iOS 12.0
3. 验证 Info.plist 中的蓝牙权限配置正确

### 蓝牙权限被拒绝
- 在 iPhone 设置 -> 隐私 -> 蓝牙 中检查应用权限
- 确保 Info.plist 中有正确的权限描述

## 示例代码位置 (Example Code)

完整的示例代码可以在以下位置找到：
- `examples/ios/DroneIDParser.swift` - 完整的 Swift 实现
- `README_iOS.md` - 详细的英文文档

## 支持和资源 (Support & Resources)

- 主仓库：https://github.com/opendroneid/opendroneid-core-c
- ASTM F3411 规范：https://www.astm.org/f3411-22a.html
- Android 接收器示例：https://github.com/opendroneid/receiver-android

## 许可证 (License)

Apache-2.0

---

有任何问题或需要帮助，请在 GitHub 上提出 issue。
