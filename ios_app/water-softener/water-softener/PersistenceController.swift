import CoreData
import Foundation

struct PersistenceController {
    static let shared = PersistenceController()

    let container: NSPersistentContainer

    init(inMemory: Bool = false) {
        container = NSPersistentContainer(name: "WaterSoftener")
        
        if inMemory {
            container.persistentStoreDescriptions.first!.url = URL(fileURLWithPath: "/dev/null")
        }
        
        container.loadPersistentStores { _, error in
            if let error = error as NSError? {
                fatalError("Unresolved error \(error), \(error.userInfo)")
            }
        }
        
        container.viewContext.automaticallyMergesChangesFromParent = true
    }
    
    func save() {
        let context = container.viewContext
        
        if context.hasChanges {
            do {
                try context.save()
            } catch {
                let nsError = error as NSError
                fatalError("Unresolved error \(nsError), \(nsError.userInfo)")
            }
        }
    }
    
    func saveSensorReading(tds: Double, temperature: Double) {
        let context = container.viewContext
        let reading = SensorReading(context: context, tds: tds, temperature: temperature)
        save()
    }
    
    func fetchRecentReadings(limit: Int = 100) -> [SensorReading] {
        let request: NSFetchRequest<SensorReading> = SensorReading.fetchRequest()
        request.sortDescriptors = [NSSortDescriptor(keyPath: \SensorReading.timestamp, ascending: false)]
        request.fetchLimit = limit
        
        do {
            return try container.viewContext.fetch(request)
        } catch {
            print("Error fetching readings: \(error)")
            return []
        }
    }
    
    func fetchReadingsForTimeRange(from startDate: Date, to endDate: Date) -> [SensorReading] {
        let request: NSFetchRequest<SensorReading> = SensorReading.fetchRequest()
        request.predicate = NSPredicate(format: "timestamp >= %@ AND timestamp <= %@", startDate as NSDate, endDate as NSDate)
        request.sortDescriptors = [NSSortDescriptor(keyPath: \SensorReading.timestamp, ascending: true)]
        
        do {
            return try container.viewContext.fetch(request)
        } catch {
            print("Error fetching readings for time range: \(error)")
            return []
        }
    }
}