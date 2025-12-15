#!/usr/bin/env python3
"""
BLE UART Logger and Web Dashboard
Reads BLE messages from serial port, formats as JSON, displays on HTML dashboard
"""

import serial
import json
import threading
import time
from datetime import datetime
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
import os

# Configuration
SERIAL_PORT = 'COM14'
BAUD_RATE = 115200
MAX_ENTRIES = 100  # Keep last 100 readings
WEB_PORT = 8000

# Global data storage
readings = deque(maxlen=MAX_ENTRIES)
readings_lock = threading.Lock()
last_update = None

def parse_ble_message(message):
    """Parse BLE message and extract temperature data"""
    try:
        # Expected format: "LED: OFF, Temp: 24.10°C"
        if "Temp:" in message:
            parts = message.split(",")
            led_state = None
            temp_value = None
            
            for part in parts:
                part = part.strip()
                if part.startswith("LED:"):
                    led_state = part.split(":")[1].strip()
                elif part.startswith("Temp:"):
                    temp_str = part.split(":")[1].strip().replace("°C", "")
                    temp_value = float(temp_str)
            
            if temp_value is not None:
                return {
                    "timestamp": datetime.now().isoformat(),
                    "led_state": led_state,
                    "temperature": temp_value,
                    "unit": "°C"
                }
    except (ValueError, IndexError, AttributeError) as e:
        print(f"Error parsing message: {message}, Error: {e}")
    
    return None

def serial_reader():
    """Read from serial port and store parsed messages"""
    global last_update
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        
        while True:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] {line}")
                        
                        # Parse and store the message
                        parsed = parse_ble_message(line)
                        if parsed:
                            with readings_lock:
                                readings.append(parsed)
                                last_update = datetime.now().isoformat()
                
                time.sleep(0.01)
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error reading serial: {e}")
                time.sleep(1)
    
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        print("Make sure the device is connected and COM port is correct")

class BLEDashboardHandler(BaseHTTPRequestHandler):
    """HTTP request handler for the web dashboard"""
    
    def do_GET(self):
        """Handle GET requests"""
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(self.get_html_dashboard().encode())
        
        elif self.path == "/api/data":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.end_headers()
            
            with readings_lock:
                data = {
                    "readings": list(readings),
                    "count": len(readings),
                    "last_update": last_update
                }
            
            self.wfile.write(json.dumps(data, indent=2).encode())
        
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        """Suppress default logging"""
        pass
    
    def get_html_dashboard(self):
        """Generate HTML dashboard"""
        return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title> Environmental mapper </title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css" />
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #0066cc 0%, #004a99 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        
        header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        
        h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .status {
            font-size: 1.1em;
            opacity: 0.9;
        }
        
        .dashboard {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.2);
        }
        
        .current-reading {
            grid-column: 1 / -1;
            background: linear-gradient(135deg, #0066cc 0%, #004a99 100%);
            color: white;
            text-align: center;
        }
        
        .current-reading h2 {
            font-size: 2em;
            margin-bottom: 20px;
        }
        
        .temp-display {
            font-size: 4em;
            font-weight: bold;
            margin-bottom: 10px;
            font-family: 'Courier New', monospace;
        }
        
        .temp-unit {
            font-size: 1.5em;
            opacity: 0.9;
        }
        
        .led-status {
            margin-top: 20px;
            font-size: 1.2em;
        }
        
        .led-on {
            color: #4CAF50;
            font-weight: bold;
        }
        
        .led-off {
            color: #f44336;
            font-weight: bold;
        }
        
        .stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        
        .stat-item {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        
        .stat-label {
            font-size: 0.9em;
            color: #666;
            margin-bottom: 5px;
        }
        
        .stat-value {
            font-size: 1.5em;
            font-weight: bold;
            color: #333;
        }
        
        .chart-container {
            grid-column: 1 / -1;
            position: relative;
            height: 300px;
        }
        
        .readings-list {
            grid-column: 1 / -1;
        }
        
        .readings-list h3 {
            margin-bottom: 15px;
            color: #333;
        }
        
        .reading-item {
            display: flex;
            justify-content: space-between;
            padding: 10px;
            border-bottom: 1px solid #eee;
            font-size: 0.9em;
        }
        
        .reading-item:hover {
            background: #f9f9f9;
        }
        
        .reading-time {
            color: #999;
            min-width: 150px;
        }
        
        .reading-temp {
            font-weight: bold;
            color: #0066cc;
            min-width: 100px;
        }
        
        .reading-led {
            color: #666;
        }
        
        .no-data {
            text-align: center;
            color: #999;
            padding: 20px;
        }
        
        .refresh-info {
            text-align: center;
            color: #999;
            font-size: 0.9em;
            margin-top: 10px;
        }
        
        @media (max-width: 768px) {
            .dashboard {
                grid-template-columns: 1fr;
            }
            
            h1 {
                font-size: 1.8em;
            }
            
            .temp-display {
                font-size: 2.5em;
            }
        }
        
        #map {
            width: 100%;
            height: 400px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        
        .map-card {
            grid-column: 1 / -1;
        }
        
        .map-card h3 {
            margin-bottom: 15px;
            color: #333;
        }
        
        .location-info {
            display: flex;
            justify-content: space-around;
            margin-top: 15px;
            padding-top: 15px;
            border-top: 1px solid #eee;
            flex-wrap: wrap;
        }
        
        .location-item {
            text-align: center;
            flex: 1;
            min-width: 150px;
        }
        
        .location-label {
            font-size: 0.9em;
            color: #666;
            margin-bottom: 5px;
        }
        
        .location-value {
            font-weight: bold;
            color: #0066cc;
            font-size: 1.1em;
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1> Environmental mapper </h1>
            <p class="status">Real-time monitoring from nRF54L15 DK</p>
        </header>
        
        <div class="dashboard">
            <div class="card current-reading">
                <h2>Current Temperature</h2>
                <div class="temp-display" id="currentTemp">--</div>
                <div class="temp-unit">°C</div>
                <div class="led-status">
                    LED State: <span id="ledStatus" class="led-off">--</span>
                </div>
                <div class="refresh-info">
                    Last update: <span id="lastUpdate">--</span>
                </div>
            </div>
            
            <div class="card">
                <h3>Statistics</h3>
                <div class="stats">
                    <div class="stat-item">
                        <div class="stat-label">Total Readings</div>
                        <div class="stat-value" id="totalReadings">0</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Average Temp</div>
                        <div class="stat-value" id="avgTemp">--</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Min Temp</div>
                        <div class="stat-value" id="minTemp">--</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Max Temp</div>
                        <div class="stat-value" id="maxTemp">--</div>
                    </div>
                </div>
            </div>
            
            <div class="card readings-list">
                <h3>Recent Readings</h3>
                <div id="readingsList" class="readings"></div>
            </div>
            
            <div class="card map-card">
                <h3>📍 Measurement Location - Trondheim, Norway</h3>
                <div id="map"></div>
                <div class="location-info">
                    <div class="location-item">
                        <div class="location-label">Latitude</div>
                        <div class="location-value">63.4305° N</div>
                    </div>
                    <div class="location-item">
                        <div class="location-label">Longitude</div>
                        <div class="location-value">10.3951° E</div>
                    </div>
                    <div class="location-item">
                        <div class="location-label">City</div>
                        <div class="location-value">Trondheim</div>
                    </div>
                    <div class="location-item">
                        <div class="location-label">Country</div>
                        <div class="location-value">Norway</div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // Initialize map
        const trondheimCoords = [63.4305, 10.3951];
        let map;
        
        function initMap() {
            map = L.map('map').setView(trondheimCoords, 13);
            
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors',
                maxZoom: 19,
                tileSize: 256
            }).addTo(map);
            
            // Add marker
            L.marker(trondheimCoords).addTo(map)
                .bindPopup('<b>nRF54L15 DK</b><br>Temperature Sensor Location')
                .openPopup();
        }
        
        // Fetch data and update dashboard
        async function updateDashboard() {
            try {
                const response = await fetch('/api/data');
                const data = await response.json();
                
                if (data.readings.length === 0) {
                    document.getElementById('readingsList').innerHTML = 
                        '<div class="no-data">Waiting for data...</div>';
                    return;
                }
                
                // Update current reading
                const latest = data.readings[data.readings.length - 1];
                document.getElementById('currentTemp').textContent = 
                    latest.temperature.toFixed(2);
                
                const ledSpan = document.getElementById('ledStatus');
                ledSpan.textContent = latest.led_state;
                ledSpan.className = latest.led_state === 'ON' ? 'led-on' : 'led-off';
                
                // Update last update time
                const updateTime = new Date(latest.timestamp);
                document.getElementById('lastUpdate').textContent = 
                    updateTime.toLocaleTimeString();
                
                // Calculate statistics
                const temps = data.readings.map(r => r.temperature);
                const avgTemp = temps.reduce((a, b) => a + b, 0) / temps.length;
                const minTemp = Math.min(...temps);
                const maxTemp = Math.max(...temps);
                
                document.getElementById('totalReadings').textContent = data.count;
                document.getElementById('avgTemp').textContent = avgTemp.toFixed(2);
                document.getElementById('minTemp').textContent = minTemp.toFixed(2);
                document.getElementById('maxTemp').textContent = maxTemp.toFixed(2);
                
                // Update readings list (show last 10)
                const recentReadings = data.readings.slice(-10).reverse();
                const listHTML = recentReadings.map(reading => {
                    const time = new Date(reading.timestamp).toLocaleTimeString();
                    return `
                        <div class="reading-item">
                            <span class="reading-time">${time}</span>
                            <span class="reading-temp">${reading.temperature.toFixed(2)}°C</span>
                            <span class="reading-led">LED: ${reading.led_state}</span>
                        </div>
                    `;
                }).join('');
                
                document.getElementById('readingsList').innerHTML = listHTML;
                
            } catch (error) {
                console.error('Error fetching data:', error);
                document.getElementById('readingsList').innerHTML = 
                    '<div class="no-data">Error loading data</div>';
            }
        }
        
        // Update every 2 seconds
        initMap();
        updateDashboard();
        setInterval(updateDashboard, 2000);
    </script>
</body>
</html>
        """

def main():
    """Main entry point"""
    print("=" * 60)
    print("BLE UART Logger - Starting...")
    print("=" * 60)
    print(f"Serial Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Web Dashboard: http://localhost:{WEB_PORT}")
    print("=" * 60)
    
    # Start serial reader thread
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    
    # Start web server
    server = HTTPServer(("", WEB_PORT), BLEDashboardHandler)
    print(f"\nWeb server running on http://localhost:{WEB_PORT}")
    print("Press Ctrl+C to stop\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        server.shutdown()

if __name__ == "__main__":
    main()
