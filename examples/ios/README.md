# iOS Example Application

This directory contains example code showing how to use the OpenDroneID framework in an iOS application.

## Swift Example

See `DroneIDParser.swift` for a complete Swift implementation that demonstrates:
- Parsing Basic ID messages
- Parsing Location messages
- Parsing Message Pack
- Bluetooth scanning for drone data
- Error handling

## Objective-C Example

See `DroneIDParser.m` and `DroneIDParser.h` for Objective-C implementations.

## Integration Steps

1. Build the framework using `build_ios_framework.sh`
2. Create a new iOS project in Xcode
3. Drag `OpenDroneID.xcframework` into your project
4. Add Bluetooth permissions to Info.plist
5. Import the framework in your code
6. Use the example code as a starting point

## Required Permissions

Add to your Info.plist:

```xml
<key>NSBluetoothAlwaysUsageDescription</key>
<string>This app needs Bluetooth to detect nearby drones</string>
<key>NSBluetoothPeripheralUsageDescription</key>
<string>This app needs Bluetooth to detect nearby drones</string>
<key>NSLocationWhenInUseUsageDescription</key>
<string>Location access is needed to show drone positions relative to you</string>
```

## Testing

You can test the framework by:
1. Building a transmitter using ESP32 or other hardware
2. Using the Android receiver app to verify transmissions
3. Testing with your iOS app

For testing without hardware, you can create mock data following the ASTM F3411 specification.
