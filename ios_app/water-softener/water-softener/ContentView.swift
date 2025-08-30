//
//  ContentView.swift
//  water-softener
//
//  Created by bryce on 6/30/25.
//

import SwiftUI
import CoreData
import Charts

struct ContentView: View {
    @Environment(\.managedObjectContext) private var viewContext
    @StateObject private var bluetoothManager = BluetoothManager()
    @FetchRequest(
        sortDescriptors: [NSSortDescriptor(keyPath: \SensorReading.timestamp, ascending: false)],
        animation: .default
    ) private var recentReadings: FetchedResults<SensorReading>
    
    @State private var selectedTimeRange: TimeRange = .hour
    
    enum TimeRange: String, CaseIterable {
        case hour = "1H"
        case day = "24H"
        case week = "7D"
        
        var timeInterval: TimeInterval {
            switch self {
            case .hour: return 3600
            case .day: return 86400
            case .week: return 604800
            }
        }
        
        var displayName: String {
            switch self {
            case .hour: return "Last Hour"
            case .day: return "Last 24 Hours"
            case .week: return "Last Week"
            }
        }
    }
    
    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 20) {
                    connectionStatusCard
                    
                    if bluetoothManager.isConnected {
                        currentReadingsCard
                        chartSection
                    }
                }
                .padding()
            }
            .navigationTitle("Water Monitor")
            .refreshable {
                if !bluetoothManager.isConnected {
                    bluetoothManager.startScanning()
                }
            }
        }
        .onAppear {
            bluetoothManager.startScanning()
        }
        .onChange(of: bluetoothManager.tdsValue) { _, newTDS in
            saveSensorReading(tds: newTDS, temperature: bluetoothManager.temperatureValue)
        }
        .onChange(of: bluetoothManager.temperatureValue) { _, newTemp in
            saveSensorReading(tds: bluetoothManager.tdsValue, temperature: newTemp)
        }
    }
    
    private var connectionStatusCard: some View {
        VStack(spacing: 12) {
            HStack {
                Circle()
                    .fill(bluetoothManager.isConnected ? .green : .red)
                    .frame(width: 12, height: 12)
                
                Text(bluetoothManager.connectionStatus)
                    .font(.headline)
                
                Spacer()
                
                if bluetoothManager.isScanning {
                    ProgressView()
                        .scaleEffect(0.8)
                }
            }
            
            HStack(spacing: 20) {
                if !bluetoothManager.isConnected {
                    Button("Scan for Device") {
                        bluetoothManager.startScanning()
                    }
                    .buttonStyle(.borderedProminent)
                } else {
                    Button("Disconnect") {
                        bluetoothManager.disconnect()
                    }
                    .buttonStyle(.bordered)
                }
                
                Spacer()
            }
        }
        .padding()
        .background(.regularMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 12))
    }
    
    private var currentReadingsCard: some View {
        VStack(spacing: 16) {
            Text("Current Readings")
                .font(.headline)
            
            HStack(spacing: 40) {
                VStack {
                    Text("TDS")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Text("\(bluetoothManager.tdsValue, specifier: "%.1f")")
                        .font(.title2)
                        .bold()
                    Text("ppm")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                
                VStack {
                    Text("Temperature")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Text("\(bluetoothManager.temperatureValue, specifier: "%.1f")")
                        .font(.title2)
                        .bold()
                    Text("°C")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
            }
        }
        .padding()
        .background(.regularMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 12))
    }
    
    private var chartSection: some View {
        VStack(spacing: 16) {
            HStack {
                Text("Historical Data")
                    .font(.headline)
                
                Spacer()
                
                Picker("Time Range", selection: $selectedTimeRange) {
                    ForEach(TimeRange.allCases, id: \.self) { range in
                        Text(range.rawValue).tag(range)
                    }
                }
                .pickerStyle(.segmented)
            }
            
            SensorChart(timeRange: selectedTimeRange)
                .frame(height: 300)
        }
        .padding()
        .background(.regularMaterial)
        .clipShape(RoundedRectangle(cornerRadius: 12))
    }
    
    private func saveSensorReading(tds: Double, temperature: Double) {
        guard tds > 0 || temperature > 0 else { return }
        
        _ = SensorReading(context: viewContext, tds: tds, temperature: temperature)
        
        do {
            try viewContext.save()
        } catch {
            print("Error saving sensor reading: \(error)")
        }
    }
}

struct SensorChart: View {
    @Environment(\.managedObjectContext) private var viewContext
    @State private var chartData: [SensorReading] = []
    
    let timeRange: ContentView.TimeRange
    
    var body: some View {
        Chart {
            ForEach(chartData, id: \.id) { reading in
                if let timestamp = reading.timestamp {
                    LineMark(
                        x: .value("Time", timestamp),
                        y: .value("TDS", reading.tds)
                    )
                    .foregroundStyle(.blue)
                    .symbol(.circle)
                    
                    LineMark(
                        x: .value("Time", timestamp),
                        y: .value("Temperature", reading.temperature * 50)
                    )
                    .foregroundStyle(.red)
                    .symbol(.square)
                }
            }
        }
        .chartYAxis {
            AxisMarks(position: .leading) { value in
                AxisValueLabel {
                    if let doubleValue = value.as(Double.self) {
                        Text("\(Int(doubleValue)) ppm")
                            .foregroundColor(.blue)
                    }
                }
                AxisGridLine()
                AxisTick()
            }
            
            AxisMarks(position: .trailing) { value in
                AxisValueLabel {
                    if let doubleValue = value.as(Double.self) {
                        Text("\(Int(doubleValue / 50))°C")
                            .foregroundColor(.red)
                    }
                }
                AxisTick()
            }
        }
        .chartLegend {
            HStack {
                Label("TDS (ppm)", systemImage: "circle.fill")
                    .foregroundColor(.blue)
                Label("Temperature (°C)", systemImage: "square.fill")
                    .foregroundColor(.red)
            }
        }
        .onAppear {
            loadChartData()
        }
        .onChange(of: timeRange) { _, _ in
            loadChartData()
        }
    }
    
    private func loadChartData() {
        let request: NSFetchRequest<SensorReading> = SensorReading.fetchRequest()
        let endDate = Date()
        let startDate = endDate.addingTimeInterval(-timeRange.timeInterval)
        
        request.predicate = NSPredicate(format: "timestamp >= %@ AND timestamp <= %@", startDate as NSDate, endDate as NSDate)
        request.sortDescriptors = [NSSortDescriptor(keyPath: \SensorReading.timestamp, ascending: true)]
        request.fetchLimit = 200
        
        do {
            chartData = try viewContext.fetch(request)
        } catch {
            print("Error fetching chart data: \(error)")
            chartData = []
        }
    }
}

#Preview {
    ContentView()
        .environment(\.managedObjectContext, PersistenceController.shared.container.viewContext)
}
