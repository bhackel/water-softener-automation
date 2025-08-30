import Foundation
import CoreData

@objc(SensorReading)
public class SensorReading: NSManagedObject {
    convenience init(context: NSManagedObjectContext, tds: Double, temperature: Double) {
        self.init(context: context)
        self.id = UUID()
        self.tds = tds
        self.temperature = temperature
        self.timestamp = Date()
    }
}