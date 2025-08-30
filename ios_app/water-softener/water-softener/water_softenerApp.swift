//
//  water_softenerApp.swift
//  water-softener
//
//  Created by bryce on 6/30/25.
//

import SwiftUI

@main
struct water_softenerApp: App {
    let persistenceController = PersistenceController.shared
    
    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(\.managedObjectContext, persistenceController.container.viewContext)
        }
    }
}
