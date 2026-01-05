//
//  DroneIDParser.swift
//  OpenDroneID iOS Example
//
//  Example Swift code for parsing Open Drone ID messages
//

import Foundation
import CoreBluetooth
import OpenDroneID

/// Parser for Open Drone ID messages
class DroneIDParser {
    
    // MARK: - Decoding Functions
    
    /// Parse Basic ID message from raw data
    /// - Parameter data: Raw message data (25 bytes)
    /// - Returns: Decoded Basic ID data or nil if parsing fails
    func parseBasicID(data: Data) -> BasicIDInfo? {
        guard data.count >= MemoryLayout<ODID_BasicID_encoded>.size else {
            print("Error: Data too short for Basic ID message")
            return nil
        }
        
        var result: BasicIDInfo?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_BasicID_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_BasicID_encoded>.size)
            
            var decoded = ODID_BasicID_data()
            let status = decodeBasicIDMessage(&decoded, &encoded)
            
            if status == ODID_SUCCESS {
                result = BasicIDInfo(from: decoded)
            } else {
                print("Failed to decode Basic ID message: \(status)")
            }
        }
        
        return result
    }
    
    /// Parse Location message from raw data
    /// - Parameter data: Raw message data (25 bytes)
    /// - Returns: Decoded Location data or nil if parsing fails
    func parseLocation(data: Data) -> LocationInfo? {
        guard data.count >= MemoryLayout<ODID_Location_encoded>.size else {
            print("Error: Data too short for Location message")
            return nil
        }
        
        var result: LocationInfo?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_Location_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_Location_encoded>.size)
            
            var decoded = ODID_Location_data()
            let status = decodeLocationMessage(&decoded, &encoded)
            
            if status == ODID_SUCCESS {
                result = LocationInfo(from: decoded)
            } else {
                print("Failed to decode Location message: \(status)")
            }
        }
        
        return result
    }
    
    /// Parse System message from raw data
    /// - Parameter data: Raw message data (25 bytes)
    /// - Returns: Decoded System data or nil if parsing fails
    func parseSystem(data: Data) -> SystemInfo? {
        guard data.count >= MemoryLayout<ODID_System_encoded>.size else {
            print("Error: Data too short for System message")
            return nil
        }
        
        var result: SystemInfo?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_System_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_System_encoded>.size)
            
            var decoded = ODID_System_data()
            let status = decodeSystemMessage(&decoded, &encoded)
            
            if status == ODID_SUCCESS {
                result = SystemInfo(from: decoded)
            } else {
                print("Failed to decode System message: \(status)")
            }
        }
        
        return result
    }
    
    /// Parse Self ID message from raw data
    /// - Parameter data: Raw message data (25 bytes)
    /// - Returns: Decoded Self ID string or nil if parsing fails
    func parseSelfID(data: Data) -> String? {
        guard data.count >= MemoryLayout<ODID_SelfID_encoded>.size else {
            print("Error: Data too short for Self ID message")
            return nil
        }
        
        var result: String?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_SelfID_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_SelfID_encoded>.size)
            
            var decoded = ODID_SelfID_data()
            let status = decodeSelfIDMessage(&decoded, &encoded)
            
            if status == ODID_SUCCESS {
                let descData = Data(bytes: &decoded.Desc, count: Int(ODID_STR_SIZE))
                result = String(data: descData, encoding: .utf8)?
                    .trimmingCharacters(in: .controlCharacters.union(.whitespaces))
            } else {
                print("Failed to decode Self ID message: \(status)")
            }
        }
        
        return result
    }
    
    /// Parse Operator ID message from raw data
    /// - Parameter data: Raw message data (25 bytes)
    /// - Returns: Decoded Operator ID string or nil if parsing fails
    func parseOperatorID(data: Data) -> String? {
        guard data.count >= MemoryLayout<ODID_OperatorID_encoded>.size else {
            print("Error: Data too short for Operator ID message")
            return nil
        }
        
        var result: String?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_OperatorID_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_OperatorID_encoded>.size)
            
            var decoded = ODID_OperatorID_data()
            let status = decodeOperatorIDMessage(&decoded, &encoded)
            
            if status == ODID_SUCCESS {
                let idData = Data(bytes: &decoded.OperatorId, count: Int(ODID_ID_SIZE))
                result = String(data: idData, encoding: .utf8)?
                    .trimmingCharacters(in: .controlCharacters.union(.whitespaces))
            } else {
                print("Failed to decode Operator ID message: \(status)")
            }
        }
        
        return result
    }
    
    /// Parse Message Pack (contains multiple message types)
    /// - Parameter data: Raw message pack data
    /// - Returns: Complete UAS data or nil if parsing fails
    func parseMessagePack(data: Data) -> UASData? {
        guard data.count >= MemoryLayout<ODID_MessagePack_encoded>.size else {
            print("Error: Data too short for Message Pack")
            return nil
        }
        
        var result: UASData?
        
        data.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
            guard let baseAddress = bytes.baseAddress else { return }
            
            var encoded = ODID_MessagePack_encoded()
            memcpy(&encoded, baseAddress, MemoryLayout<ODID_MessagePack_encoded>.size)
            
            var uasData = ODID_UAS_Data()
            let status = decodeMessagePack(&uasData, &encoded)
            
            if status == ODID_SUCCESS {
                result = UASData(from: uasData)
            } else {
                print("Failed to decode Message Pack: \(status)")
            }
        }
        
        return result
    }
    
    /// Determine message type from raw data
    /// - Parameter data: Raw message data
    /// - Returns: Message type enum value
    func getMessageType(data: Data) -> ODID_messagetype? {
        guard data.count > 0 else { return nil }
        
        let firstByte = data[0]
        let messageType = (firstByte >> 4) & 0x0F
        
        return ODID_messagetype(rawValue: UInt32(messageType))
    }
}

// MARK: - Data Models

/// Parsed Basic ID information
struct BasicIDInfo {
    let uaType: ODID_uatype
    let idType: ODID_idtype
    let uasID: String
    
    init(from data: ODID_BasicID_data) {
        self.uaType = data.UAType
        self.idType = data.IDType
        
        // Safely extract UASID using withUnsafeBytes
        var uasidCopy = data.UASID
        let idData = withUnsafeBytes(of: &uasidCopy) { bytes in
            Data(bytes: bytes.baseAddress!, count: Int(ODID_ID_SIZE))
        }
        self.uasID = String(data: idData, encoding: .utf8)?
            .trimmingCharacters(in: .controlCharacters.union(.whitespaces)) ?? ""
    }
}

/// Parsed Location information
struct LocationInfo {
    let status: ODID_status
    let latitude: Double
    let longitude: Double
    let altitudeBarometric: Float
    let altitudeGeodetic: Float
    let height: Float
    let horizontalSpeed: Float
    let verticalSpeed: Float
    let direction: Float
    let timestamp: Float
    
    init(from data: ODID_Location_data) {
        self.status = data.Status
        self.latitude = data.Latitude
        self.longitude = data.Longitude
        self.altitudeBarometric = data.AltitudeBaro
        self.altitudeGeodetic = data.AltitudeGeo
        self.height = data.Height
        self.horizontalSpeed = data.SpeedHorizontal
        self.verticalSpeed = data.SpeedVertical
        self.direction = data.Direction
        self.timestamp = data.TimeStamp
    }
}

/// Parsed System information
struct SystemInfo {
    let operatorLatitude: Double
    let operatorLongitude: Double
    let operatorAltitude: Float
    let areaCount: UInt16
    let areaRadius: UInt16
    let areaCeiling: Float
    let areaFloor: Float
    let timestamp: UInt32
    
    init(from data: ODID_System_data) {
        self.operatorLatitude = data.OperatorLatitude
        self.operatorLongitude = data.OperatorLongitude
        self.operatorAltitude = data.OperatorAltitudeGeo
        self.areaCount = data.AreaCount
        self.areaRadius = data.AreaRadius
        self.areaCeiling = data.AreaCeiling
        self.areaFloor = data.AreaFloor
        self.timestamp = data.Timestamp
    }
}

/// Complete UAS (Unmanned Aircraft System) data
struct UASData {
    let basicID: [BasicIDInfo]
    let location: LocationInfo?
    let system: SystemInfo?
    let selfID: String?
    let operatorID: String?
    
    init(from data: ODID_UAS_Data) {
        var basicIDs: [BasicIDInfo] = []
        
        // Extract valid Basic IDs
        for i in 0..<Int(ODID_BASIC_ID_MAX_MESSAGES) {
            if data.BasicIDValid[i] != 0 {
                let basicIDData = data.BasicID[i]
                basicIDs.append(BasicIDInfo(from: basicIDData))
            }
        }
        
        self.basicID = basicIDs
        
        // Extract Location if valid
        if data.LocationValid != 0 {
            self.location = LocationInfo(from: data.Location)
        } else {
            self.location = nil
        }
        
        // Extract System if valid
        if data.SystemValid != 0 {
            self.system = SystemInfo(from: data.System)
        } else {
            self.system = nil
        }
        
        // Extract Self ID if valid
        if data.SelfIDValid != 0 {
            var descCopy = data.SelfID.Desc
            let dataObj = withUnsafeBytes(of: &descCopy) { bytes in
                Data(bytes: bytes.baseAddress!, count: Int(ODID_STR_SIZE))
            }
            self.selfID = String(data: dataObj, encoding: .utf8)?
                .trimmingCharacters(in: .controlCharacters.union(.whitespaces))
        } else {
            self.selfID = nil
        }
        
        // Extract Operator ID if valid
        if data.OperatorIDValid != 0 {
            var operatorIdCopy = data.OperatorID.OperatorId
            let dataObj = withUnsafeBytes(of: &operatorIdCopy) { bytes in
                Data(bytes: bytes.baseAddress!, count: Int(ODID_ID_SIZE))
            }
            self.operatorID = String(data: dataObj, encoding: .utf8)?
                .trimmingCharacters(in: .controlCharacters.union(.whitespaces))
        } else {
            self.operatorID = nil
        }
    }
}

// MARK: - Bluetooth Scanner

/// Bluetooth scanner for detecting Open Drone ID broadcasts
class DroneIDBluetoothScanner: NSObject, CBCentralManagerDelegate {
    private var centralManager: CBCentralManager!
    private let parser = DroneIDParser()
    
    /// Callback for when a drone is detected
    var onDroneDetected: ((UASData) -> Void)?
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func startScanning() {
        guard centralManager.state == .poweredOn else {
            print("Bluetooth is not powered on")
            return
        }
        
        // Scan for all peripherals
        centralManager.scanForPeripherals(
            withServices: nil,
            options: [CBCentralManagerScanOptionAllowDuplicatesKey: true]
        )
        print("Started scanning for drones...")
    }
    
    func stopScanning() {
        centralManager.stopScan()
        print("Stopped scanning")
    }
    
    // MARK: - CBCentralManagerDelegate
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("Bluetooth powered on")
            startScanning()
        case .poweredOff:
            print("Bluetooth powered off")
        case .unauthorized:
            print("Bluetooth unauthorized")
        case .unsupported:
            print("Bluetooth not supported")
        default:
            break
        }
    }
    
    func centralManager(_ central: CBCentralManager,
                       didDiscover peripheral: CBPeripheral,
                       advertisementData: [String : Any],
                       rssi RSSI: NSNumber) {
        
        // Check for Open Drone ID service data
        if let serviceData = advertisementData[CBAdvertisementDataServiceDataKey] as? [CBUUID: Data] {
            for (uuid, data) in serviceData {
                processDroneData(data, rssi: RSSI.intValue)
            }
        }
        
        // Check manufacturer data (alternative transmission method)
        if let manufacturerData = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data {
            processDroneData(manufacturerData, rssi: RSSI.intValue)
        }
    }
    
    private func processDroneData(_ data: Data, rssi: Int) {
        // Try to parse as Message Pack first
        if let uasData = parser.parseMessagePack(data: data) {
            print("Detected drone (RSSI: \(rssi))")
            onDroneDetected?(uasData)
            return
        }
        
        // Otherwise, try to determine message type and parse individually
        if let messageType = parser.getMessageType(data: data) {
            switch messageType {
            case ODID_MESSAGETYPE_BASIC_ID:
                if let basicID = parser.parseBasicID(data: data) {
                    print("Basic ID: \(basicID.uasID)")
                }
            case ODID_MESSAGETYPE_LOCATION:
                if let location = parser.parseLocation(data: data) {
                    print("Location: \(location.latitude), \(location.longitude)")
                }
            case ODID_MESSAGETYPE_SYSTEM:
                if let system = parser.parseSystem(data: data) {
                    print("System data received")
                }
            default:
                break
            }
        }
    }
}

// MARK: - Example Usage

/*
 Example usage in a view controller:
 
 class ViewController: UIViewController {
     let scanner = DroneIDBluetoothScanner()
     
     override func viewDidLoad() {
         super.viewDidLoad()
         
         scanner.onDroneDetected = { uasData in
             // Update UI with drone data
             print("Drone detected!")
             
             if let basicID = uasData.basicID.first {
                 print("ID: \(basicID.uasID)")
             }
             
             if let location = uasData.location {
                 print("Position: \(location.latitude), \(location.longitude)")
                 print("Altitude: \(location.altitudeBarometric) m")
                 print("Speed: \(location.horizontalSpeed) m/s")
             }
         }
     }
     
     @IBAction func startScanningTapped(_ sender: Any) {
         scanner.startScanning()
     }
     
     @IBAction func stopScanningTapped(_ sender: Any) {
         scanner.stopScanning()
     }
 }
 */
