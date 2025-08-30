import Foundation
import CoreData

extension SensorReading {
    @nonobjc public class func fetchRequest() -> NSFetchRequest<SensorReading> {
        return NSFetchRequest<SensorReading>(entityName: "SensorReading")
    }

    @NSManaged public var id: UUID?
    @NSManaged public var tds: Double
    @NSManaged public var temperature: Double
    @NSManaged public var timestamp: Date?
}