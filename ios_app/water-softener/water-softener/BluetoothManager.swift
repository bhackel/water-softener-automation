import Foundation
import CoreBluetooth
import Combine

class BluetoothManager: NSObject, ObservableObject {
    @Published var isConnected = false
    @Published var isScanning = false
    @Published var tdsValue: Double = 0.0
    @Published var temperatureValue: Double = 0.0
    @Published var connectionStatus = "Disconnected"
    
    private var centralManager: CBCentralManager!
    private var waterDevice: CBPeripheral?
    private var tdsCharacteristic: CBCharacteristic?
    private var temperatureCharacteristic: CBCharacteristic?
    
    private let waterDeviceName = "Water"
    private let serviceUUID = CBUUID(string: "12345678-1234-5678-9abc-def012345678")
    private let tdsCharacteristicUUID = CBUUID(string: "12345678-1234-5678-9abc-def012345679")
    private let temperatureCharacteristicUUID = CBUUID(string: "12345678-1234-5678-9abc-def01234567a")
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func startScanning() {
        guard centralManager.state == .poweredOn else {
            connectionStatus = "Bluetooth not available"
            return
        }
        
        isScanning = true
        connectionStatus = "Scanning..."
        centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 10) {
            if self.isScanning {
                self.stopScanning()
                self.connectionStatus = "Device not found"
            }
        }
    }
    
    func stopScanning() {
        isScanning = false
        centralManager.stopScan()
    }
    
    func disconnect() {
        guard let device = waterDevice else { return }
        centralManager.cancelPeripheralConnection(device)
    }
}

extension BluetoothManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            connectionStatus = "Ready to scan"
        case .poweredOff:
            connectionStatus = "Bluetooth is off"
        case .unauthorized:
            connectionStatus = "Bluetooth unauthorized"
        case .unsupported:
            connectionStatus = "Bluetooth unsupported"
        default:
            connectionStatus = "Bluetooth unavailable"
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        if peripheral.name == waterDeviceName {
            stopScanning()
            waterDevice = peripheral
            waterDevice?.delegate = self
            connectionStatus = "Connecting..."
            centralManager.connect(peripheral, options: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        isConnected = true
        connectionStatus = "Connected"
        peripheral.discoverServices([serviceUUID])
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        connectionStatus = "Failed to connect"
        isConnected = false
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        isConnected = false
        connectionStatus = "Disconnected"
        waterDevice = nil
        tdsCharacteristic = nil
        temperatureCharacteristic = nil
    }
}

extension BluetoothManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        
        for service in services {
            if service.uuid == serviceUUID {
                peripheral.discoverCharacteristics([tdsCharacteristicUUID, temperatureCharacteristicUUID], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            if characteristic.uuid == tdsCharacteristicUUID {
                tdsCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            } else if characteristic.uuid == temperatureCharacteristicUUID {
                temperatureCharacteristic = characteristic
                peripheral.setNotifyValue(true, for: characteristic)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }
        
        if characteristic.uuid == tdsCharacteristicUUID {
            let value = data.withUnsafeBytes { $0.load(as: Float32.self) }
            DispatchQueue.main.async {
                self.tdsValue = Double(value)
            }
        } else if characteristic.uuid == temperatureCharacteristicUUID {
            let value = data.withUnsafeBytes { $0.load(as: Float32.self) }
            DispatchQueue.main.async {
                self.temperatureValue = Double(value)
            }
        }
    }
}