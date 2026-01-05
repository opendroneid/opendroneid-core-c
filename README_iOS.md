# OpenDroneID iOS Framework

This document describes how to build and use the OpenDroneID Core C Library as an iOS framework.

## Overview

The OpenDroneID library provides encoding and decoding functions for Open Drone ID messages as defined in the ASTM F3411 Remote ID and ASD-STAN prEN 4709-002 Direct Remote ID specifications. This iOS framework allows you to parse drone data received via Bluetooth or Wi-Fi on iOS devices.

## Building the Framework

### Requirements

- macOS with Xcode installed
- Xcode Command Line Tools
- iOS SDK (minimum iOS 12.0)

### Build Instructions

1. Clone the repository and navigate to the project directory:
```bash
git clone https://github.com/honghen28hshyan/opendroneid-core-c.git
cd opendroneid-core-c
```

2. (Optional) Check your build environment:
```bash
./check_ios_env.sh
```

This script will verify that you have all required tools installed.

3. Run the build script:
```bash
./build_ios_framework.sh
```

4. The script will create an XCFramework at `output/OpenDroneID.xcframework`

The build script:
- Compiles the library for iOS devices (arm64)
- Compiles the library for iOS Simulator (arm64 and x86_64)
- Creates frameworks for each platform
- Packages everything into a universal XCFramework

## Using the Framework in Your iOS Project

### Integration

1. Drag and drop `OpenDroneID.xcframework` into your Xcode project
2. Ensure the framework is added to your target's "Frameworks, Libraries, and Embedded Content" section
3. Set "Embed" to "Embed & Sign" or "Do Not Embed" (if you prefer static linking)

### Swift Usage

```swift
import OpenDroneID

class DroneIDParser {
    func parseBasicID(data: Data) {
        // Convert Data to byte array
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            // Decode Basic ID message
            var encoded = ODID_BasicID_encoded()
            memcpy(&encoded, baseAddress, min(bytes.count, MemoryLayout<ODID_BasicID_encoded>.size))
            
            var decoded = ODID_BasicID_data()
            let result = decodeBasicIDMessage(&decoded, &encoded)
            
            if result == ODID_SUCCESS {
                // Successfully decoded
                print("UA Type: \(decoded.UAType)")
                print("ID Type: \(decoded.IDType)")
                
                // Get ID string
                let idData = Data(bytes: &decoded.UASID, count: Int(ODID_ID_SIZE))
                if let idString = String(data: idData, encoding: .utf8) {
                    print("UAS ID: \(idString)")
                }
            }
        }
    }
    
    func parseLocation(data: Data) {
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_Location_encoded()
            memcpy(&encoded, baseAddress, min(bytes.count, MemoryLayout<ODID_Location_encoded>.size))
            
            var decoded = ODID_Location_data()
            let result = decodeLocationMessage(&decoded, &encoded)
            
            if result == ODID_SUCCESS {
                print("Latitude: \(decoded.Latitude)")
                print("Longitude: \(decoded.Longitude)")
                print("Altitude: \(decoded.AltitudeBaro)")
                print("Speed: \(decoded.SpeedHorizontal)")
                print("Direction: \(decoded.Direction)")
            }
        }
    }
    
    func parseMessagePack(data: Data) {
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_MessagePack_encoded()
            memcpy(&encoded, baseAddress, min(bytes.count, MemoryLayout<ODID_MessagePack_encoded>.size))
            
            var uasData = ODID_UAS_Data()
            let result = decodeMessagePack(&uasData, &encoded)
            
            if result == ODID_SUCCESS {
                // Access all message types
                print("Basic ID valid: \(uasData.BasicIDValid)")
                print("Location valid: \(uasData.LocationValid)")
                print("System valid: \(uasData.SystemValid)")
            }
        }
    }
}
```

### Objective-C Usage

```objective-c
#import <OpenDroneID/OpenDroneID.h>

@implementation DroneIDParser

- (void)parseBasicID:(NSData *)data {
    ODID_BasicID_encoded encoded;
    [data getBytes:&encoded length:MIN(data.length, sizeof(ODID_BasicID_encoded))];
    
    ODID_BasicID_data decoded;
    int result = decodeBasicIDMessage(&decoded, &encoded);
    
    if (result == ODID_SUCCESS) {
        NSLog(@"UA Type: %d", decoded.UAType);
        NSLog(@"ID Type: %d", decoded.IDType);
        
        NSString *uasID = [[NSString alloc] initWithBytes:decoded.UASID
                                                   length:ODID_ID_SIZE
                                                 encoding:NSUTF8StringEncoding];
        NSLog(@"UAS ID: %@", uasID);
    }
}

- (void)parseLocation:(NSData *)data {
    ODID_Location_encoded encoded;
    [data getBytes:&encoded length:MIN(data.length, sizeof(ODID_Location_encoded))];
    
    ODID_Location_data decoded;
    int result = decodeLocationMessage(&decoded, &encoded);
    
    if (result == ODID_SUCCESS) {
        NSLog(@"Latitude: %f", decoded.Latitude);
        NSLog(@"Longitude: %f", decoded.Longitude);
        NSLog(@"Altitude: %f", decoded.AltitudeBaro);
        NSLog(@"Speed: %f", decoded.SpeedHorizontal);
        NSLog(@"Direction: %f", decoded.Direction);
    }
}

@end
```

## Message Types

The OpenDroneID library supports the following message types:

### Basic ID Message
Contains identification information about the drone:
- UAS ID (serial number or session ID)
- UA type (helicopter, fixed wing, etc.)
- ID type (serial number, CAA assigned, etc.)

### Location Message
Contains real-time location and movement data:
- Latitude and Longitude
- Altitude (barometric, geodetic, AGL)
- Speed (horizontal and vertical)
- Direction
- Timestamp

### System Message
Contains information about the operator and system:
- Operator location
- Area count, radius, ceiling, floor
- Classification type
- Timestamp

### Self ID Message
Contains descriptive text set by the operator

### Operator ID Message
Contains the operator registration ID

### Authentication Message
Contains authentication data for verifying drone identity

### Message Pack
A container that can hold multiple messages of different types

## Key Functions

### Decoding Functions
```c
int decodeBasicIDMessage(ODID_BasicID_data *outData, const ODID_BasicID_encoded *inEncoded);
int decodeLocationMessage(ODID_Location_data *outData, const ODID_Location_encoded *inEncoded);
int decodeAuthMessage(ODID_Auth_data *outData, const ODID_Auth_encoded *inEncoded);
int decodeSelfIDMessage(ODID_SelfID_data *outData, const ODID_SelfID_encoded *inEncoded);
int decodeSystemMessage(ODID_System_data *outData, const ODID_System_encoded *inEncoded);
int decodeOperatorIDMessage(ODID_OperatorID_data *outData, const ODID_OperatorID_encoded *inEncoded);
int decodeMessagePack(ODID_UAS_Data *uasData, const ODID_MessagePack_encoded *pack);
```

### Encoding Functions
```c
int encodeBasicIDMessage(ODID_BasicID_encoded *outEncoded, const ODID_BasicID_data *inData);
int encodeLocationMessage(ODID_Location_encoded *outEncoded, const ODID_Location_data *inData);
int encodeAuthMessage(ODID_Auth_encoded *outEncoded, const ODID_Auth_data *inData);
int encodeSelfIDMessage(ODID_SelfID_encoded *outEncoded, const ODID_SelfID_data *inData);
int encodeSystemMessage(ODID_System_encoded *outEncoded, const ODID_System_data *inData);
int encodeOperatorIDMessage(ODID_OperatorID_encoded *outEncoded, const ODID_OperatorID_data *inData);
int encodeMessagePack(ODID_MessagePack_encoded *outEncoded, const ODID_MessagePack_data *inData);
```

## Bluetooth Data Reception

To receive drone data via Bluetooth on iOS, you'll need to:

1. Use Core Bluetooth framework to scan for advertisements
2. Look for service UUID or manufacturer data containing drone ID
3. Extract the payload data
4. Pass it to the OpenDroneID decoding functions

Example Bluetooth scanning code:

```swift
import CoreBluetooth

class DroneScanner: NSObject, CBCentralManagerDelegate {
    var centralManager: CBCentralManager!
    
    func startScanning() {
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            // Start scanning for all peripherals
            central.scanForPeripherals(withServices: nil, options: [
                CBCentralManagerScanOptionAllowDuplicatesKey: true
            ])
        }
    }
    
    func centralManager(_ central: CBCentralManager, 
                       didDiscover peripheral: CBPeripheral,
                       advertisementData: [String : Any], 
                       rssi RSSI: NSNumber) {
        // Check for Open Drone ID service data
        if let serviceData = advertisementData[CBAdvertisementDataServiceDataKey] as? [CBUUID: Data] {
            for (uuid, data) in serviceData {
                // Process drone ID data
                parseDroneData(data)
            }
        }
        
        // Check manufacturer data
        if let manufacturerData = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data {
            parseDroneData(manufacturerData)
        }
    }
    
    func parseDroneData(_ data: Data) {
        // Use OpenDroneID library to decode
        // Determine message type from first byte and decode accordingly
    }
}
```

## Wi-Fi Data Reception

The library also includes functions for Wi-Fi Beacon and NaN (Neighbor Awareness Networking) frame parsing. However, iOS currently has limited API access for Wi-Fi frame monitoring.

## Constants and Limits

```c
#define ODID_MESSAGE_SIZE 25        // Size of each message
#define ODID_ID_SIZE 20             // Size of ID fields
#define ODID_STR_SIZE 23            // Size of string fields
#define ODID_SUCCESS 0              // Success return code
#define ODID_FAIL 1                 // Failure return code
```

## Troubleshooting

### Build Issues
- Ensure Xcode Command Line Tools are installed: `xcode-select --install`
- Check that you have the latest Xcode version
- Verify iOS SDK is available: `xcodebuild -showsdks`

### Runtime Issues
- Ensure the framework is properly embedded in your app
- Check that your iOS deployment target is at least iOS 12.0
- Verify you have proper Bluetooth permissions in Info.plist

### Bluetooth Permissions
Add these keys to your Info.plist:
```xml
<key>NSBluetoothAlwaysUsageDescription</key>
<string>This app needs Bluetooth to detect nearby drones</string>
<key>NSBluetoothPeripheralUsageDescription</key>
<string>This app needs Bluetooth to detect nearby drones</string>
```

## Standards Compliance

This library is compliant with:
- ASTM F3411-22a (Remote ID and Tracking)
- ASD-STAN prEN 4709-002 (Direct Remote Identification)

Protocol version: 2

## License

Apache-2.0

## Support

For issues and questions:
- Main repository: https://github.com/opendroneid/opendroneid-core-c
- ASTM F3411 specification: https://www.astm.org/f3411-22a.html

## Additional Resources

- [Receiver Android App](https://github.com/opendroneid/receiver-android)
- [Transmitter Examples](https://github.com/opendroneid/opendroneid-core-c#transmitter-examples)
- [MAVLink OpenDroneID Messages](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID)
