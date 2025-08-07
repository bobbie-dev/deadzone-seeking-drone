#!/usr/bin/env python3
"""
Enhanced Mission Control Software with Cellular Data Monitoring
Integrates with Mission-Triggered Cellular Data Logger
"""

import sys, time, struct, json, os
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QVBoxLayout, QWidget,
    QPushButton, QSpinBox, QLabel, QHBoxLayout, QMessageBox, QTextEdit,
    QGroupBox, QGridLayout, QTabWidget, QTableWidget, QTableWidgetItem,
    QProgressBar, QFrame, QSplitter, QComboBox
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QPixmap, QPainter, QPen, QBrush, QColor
import folium
import io
from PyQt5.QtWebEngineWidgets import QWebEngineView


class DronePositionThread(QThread):
    position_update = pyqtSignal(float, float, float)  # lat, lon, alt
    connection_status = pyqtSignal(bool, str)  # connected, status_message

    def __init__(self, port='COM4', baud=57600):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self.master = None

    def run(self):
        from pymavlink import mavutil
        try:
            self.connection_status.emit(False, f"Connecting to {self.port}...")
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.master.wait_heartbeat(timeout=10)
            
            status_msg = f"Connected to system {self.master.target_system}, component {self.master.target_component}"
            print(status_msg)
            self.connection_status.emit(True, status_msg)
            
        except Exception as e:
            error_msg = f"Drone connection error: {e}"
            print(error_msg)
            self.connection_status.emit(False, error_msg)
            return
            
        while self._running:
            try:
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
                if msg:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000
                    self.position_update.emit(lat, lon, alt)
            except Exception as e:
                if self._running:  # Only log if we're still supposed to be running
                    print(f"Error receiving drone position: {e}")
                continue

    def stop(self):
        self._running = False
        if self.master:
            try:
                self.master.close()
            except:
                pass
        self.wait()


class CellularDataThread(QThread):
    cellular_data_received = pyqtSignal(dict)  # Parsed cellular data
    mission_status_update = pyqtSignal(str, dict)  # status_type, data
    raw_message_received = pyqtSignal(str, str)  # message_type, content

    def __init__(self, master):
        super().__init__()
        self.master = master
        self._running = True

    def run(self):
        while self._running and self.master:
            try:
                # Listen for various message types
                msg = self.master.recv_match(
                    type=['STATUSTEXT', 'RADIO_STATUS', 'PARAM_VALUE', 'CUSTOM_MSG'], 
                    blocking=True, 
                    timeout=2
                )
                
                if msg:
                    msg_type = msg.get_type()
                    
                    if msg_type == 'STATUSTEXT':
                        self.handle_status_text(msg)
                    elif msg_type == 'RADIO_STATUS':
                        self.handle_radio_status(msg)
                    elif msg_type == 'PARAM_VALUE':
                        self.handle_param_value(msg)
                    elif msg_type == 'CUSTOM_MSG':
                        self.handle_custom_message(msg)
                        
            except Exception as e:
                if self._running:
                    print(f"Error receiving cellular data: {e}")
                continue

    def handle_status_text(self, msg):
        """Handle status text messages from drone"""
        try:
            text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
            text = text.strip()
            
            self.raw_message_received.emit('STATUSTEXT', text)
            
            # Parse cellular data logger messages
            if text.startswith('CELL_LOG_START:'):
                mission_id = text.split(':')[1]
                self.mission_status_update.emit('mission_started', {'mission_id': mission_id})
                
            elif text.startswith('CELL_LOG_STOP:'):
                data_points = text.split(':')[1]
                self.mission_status_update.emit('mission_stopped', {'data_points': int(data_points)})
                
            elif text.startswith('CD:'):  # Cellular data transmission
                parts = text.split(':')
                if len(parts) >= 3:
                    packet_num = parts[1]
                    packet_size = parts[2]
                    self.cellular_data_received.emit({
                        'packet_num': packet_num,
                        'packet_size': packet_size,
                        'timestamp': time.time()
                    })
                    
        except Exception as e:
            print(f"Error handling status text: {e}")

    def handle_radio_status(self, msg):
        """Handle radio status messages (used for cellular data transmission)"""
        try:
            # Our cellular logger uses radio status for compact transmission
            data = {
                'rssi': msg.rssi,
                'packet_size': msg.rssi,  # We encoded packet size in rssi field
                'packet_counter': msg.remrssi,
                'retry_attempt': msg.rxerrors,
                'timestamp': time.time()
            }
            self.cellular_data_received.emit(data)
        except Exception as e:
            print(f"Error handling radio status: {e}")

    def handle_param_value(self, msg):
        """Handle parameter value messages (cellular data)"""
        try:
            param_name = msg.param_id.decode('utf-8').strip('\x00')
            if param_name.startswith('CD'):  # Cellular Data parameter
                data = {
                    'param_name': param_name,
                    'value': msg.param_value,
                    'packet_index': msg.param_index,
                    'total_params': msg.param_count,
                    'timestamp': time.time()
                }
                self.cellular_data_received.emit(data)
        except Exception as e:
            print(f"Error handling param value: {e}")

    def handle_custom_message(self, msg):
        """Handle custom MAVLink messages"""
        try:
            # Unpack cellular data from custom message
            if hasattr(msg, 'Data') and len(msg.Data) >= 17:  # Our compact packet size
                data = struct.unpack('!iiHBBBHH', msg.Data[:17])
                
                cellular_info = {
                    'latitude': data[0] / 1e6,
                    'longitude': data[1] / 1e6, 
                    'altitude': data[2],
                    'rssi': data[3] - 150,  # Decompress
                    'rsrp': data[4] - 180,  # Decompress
                    'rsrq': (data[5] / 4.0) - 20,  # Decompress
                    'cell_id': data[6],
                    'packet_counter': data[7],
                    'timestamp': time.time()
                }
                
                self.cellular_data_received.emit(cellular_info)
        except Exception as e:
            print(f"Error handling custom message: {e}")

    def stop(self):
        self._running = False
        self.wait()


class CellularDataWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.cellular_data_log = []
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Mission Status
        mission_group = QGroupBox("Mission Status")
        mission_layout = QGridLayout()
        
        self.mission_status_label = QLabel("Status: Idle")
        self.mission_id_label = QLabel("Mission ID: N/A")
        self.data_points_label = QLabel("Data Points: 0")
        self.last_transmission_label = QLabel("Last TX: N/A")
        
        mission_layout.addWidget(self.mission_status_label, 0, 0)
        mission_layout.addWidget(self.mission_id_label, 0, 1)
        mission_layout.addWidget(self.data_points_label, 1, 0)
        mission_layout.addWidget(self.last_transmission_label, 1, 1)
        
        mission_group.setLayout(mission_layout)
        layout.addWidget(mission_group)
        
        # Current Cellular Data
        cellular_group = QGroupBox("Current Cellular Data")
        cellular_layout = QGridLayout()
        
        self.rssi_label = QLabel("RSSI: N/A")
        self.rsrp_label = QLabel("RSRP: N/A") 
        self.rsrq_label = QLabel("RSRQ: N/A")
        self.cell_id_label = QLabel("Cell ID: N/A")
        
        cellular_layout.addWidget(self.rssi_label, 0, 0)
        cellular_layout.addWidget(self.rsrp_label, 0, 1)
        cellular_layout.addWidget(self.rsrq_label, 1, 0)
        cellular_layout.addWidget(self.cell_id_label, 1, 1)
        
        cellular_group.setLayout(cellular_layout)
        layout.addWidget(cellular_group)
        
        # Data Log Table
        log_group = QGroupBox("Recent Cellular Data")
        log_layout = QVBoxLayout()
        
        self.data_table = QTableWidget()
        self.data_table.setColumnCount(6)
        self.data_table.setHorizontalHeaderLabels(['Time', 'RSSI', 'RSRP', 'RSRQ', 'Cell ID', 'Location'])
        self.data_table.setMaximumHeight(200)
        
        log_layout.addWidget(self.data_table)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        self.setLayout(layout)
    
    def update_mission_status(self, status_type, data):
        """Update mission status display"""
        if status_type == 'mission_started':
            self.mission_status_label.setText("Status: Mission Active")
            self.mission_status_label.setStyleSheet("color: green; font-weight: bold;")
            self.mission_id_label.setText(f"Mission ID: {data['mission_id']}")
            
        elif status_type == 'mission_stopped':
            self.mission_status_label.setText("Status: Mission Complete")
            self.mission_status_label.setStyleSheet("color: orange; font-weight: bold;")
            self.data_points_label.setText(f"Data Points: {data['data_points']}")
    
    def update_cellular_data(self, data):
        """Update cellular data display"""
        # Update current readings
        if 'rssi' in data:
            self.rssi_label.setText(f"RSSI: {data['rssi']:.0f} dBm")
        if 'rsrp' in data:
            self.rsrp_label.setText(f"RSRP: {data['rsrp']:.0f} dBm") 
        if 'rsrq' in data:
            self.rsrq_label.setText(f"RSRQ: {data['rsrq']:.1f} dB")
        if 'cell_id' in data:
            self.cell_id_label.setText(f"Cell ID: {data['cell_id']}")
            
        # Update last transmission time
        if 'timestamp' in data:
            time_str = datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')
            self.last_transmission_label.setText(f"Last TX: {time_str}")
        
        # Add to data log
        self.cellular_data_log.append(data)
        self.update_data_table()
    
    def update_data_table(self):
        """Update the data table with recent entries"""
        recent_data = self.cellular_data_log[-20:]  # Show last 20 entries
        self.data_table.setRowCount(len(recent_data))
        
        for row, data in enumerate(recent_data):
            # Time
            if 'timestamp' in data:
                time_str = datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S')
                self.data_table.setItem(row, 0, QTableWidgetItem(time_str))
            
            # RSSI
            if 'rssi' in data:
                self.data_table.setItem(row, 1, QTableWidgetItem(f"{data['rssi']:.0f}"))
            
            # RSRP  
            if 'rsrp' in data:
                self.data_table.setItem(row, 2, QTableWidgetItem(f"{data['rsrp']:.0f}"))
            
            # RSRQ
            if 'rsrq' in data:
                self.data_table.setItem(row, 3, QTableWidgetItem(f"{data['rsrq']:.1f}"))
            
            # Cell ID
            if 'cell_id' in data:
                self.data_table.setItem(row, 4, QTableWidgetItem(str(data['cell_id'])))
            
            # Location
            if 'latitude' in data and 'longitude' in data:
                loc_str = f"{data['latitude']:.5f}, {data['longitude']:.5f}"
                self.data_table.setItem(row, 5, QTableWidgetItem(loc_str))


class MapWidget(QWidget):
    def __init__(self):
        super().__init__()
        # Initialize data lists before setup_ui
        self.drone_positions = []
        self.cellular_data_points = []
        self.setup_ui()
        
    def setup_ui(self):
        layout = QVBoxLayout()
        
        # Map controls
        controls = QHBoxLayout()
        self.center_on_drone_btn = QPushButton("Center on Drone")
        self.clear_trail_btn = QPushButton("Clear Trail")
        self.export_map_btn = QPushButton("Export Map")
        
        controls.addWidget(self.center_on_drone_btn)
        controls.addWidget(self.clear_trail_btn)
        controls.addWidget(self.export_map_btn)
        controls.addStretch()
        
        layout.addLayout(controls)
        
        # Web view for map
        self.web_view = QWebEngineView()
        layout.addWidget(self.web_view)
        
        self.setLayout(layout)
        
        # Connect buttons
        self.center_on_drone_btn.clicked.connect(self.center_on_drone)
        self.clear_trail_btn.clicked.connect(self.clear_trail)
        self.export_map_btn.clicked.connect(self.export_map)
        
        # Initialize map
        self.update_map()
    
    def update_drone_position(self, lat, lon, alt):
        """Update drone position on map"""
        self.drone_positions.append({'lat': lat, 'lon': lon, 'alt': alt, 'time': time.time()})
        
        # Keep only last 1000 positions
        if len(self.drone_positions) > 1000:
            self.drone_positions = self.drone_positions[-1000:]
            
        self.update_map()
    
    def add_cellular_data_point(self, data):
        """Add cellular data point to map"""
        if 'latitude' in data and 'longitude' in data:
            self.cellular_data_points.append(data)
            self.update_map()
    
    def update_map(self):
        """Update the folium map"""
        # Default center
        center_lat, center_lon = 29.5994, -95.6306  # Sugar Land, TX
        
        # Use drone position if available
        if self.drone_positions:
            latest_pos = self.drone_positions[-1]
            center_lat, center_lon = latest_pos['lat'], latest_pos['lon']
        
        # Create map
        m = folium.Map(location=[center_lat, center_lon], zoom_start=15)
        
        # Add drone trail
        if len(self.drone_positions) > 1:
            trail_coords = [[pos['lat'], pos['lon']] for pos in self.drone_positions]
            folium.PolyLine(trail_coords, color='blue', weight=3, opacity=0.7).add_to(m)
        
        # Add current drone position
        if self.drone_positions:
            current = self.drone_positions[-1]
            folium.Marker(
                [current['lat'], current['lon']], 
                popup=f"Drone<br>Alt: {current['alt']:.1f}m<br>Time: {datetime.fromtimestamp(current['time']).strftime('%H:%M:%S')}",
                icon=folium.Icon(color='red', icon='plane')
            ).add_to(m)
        
        # Add cellular data points
        for data in self.cellular_data_points[-100:]:  # Show last 100 points
            if 'latitude' in data and 'longitude' in data:
                # Color code by signal strength
                rssi = data.get('rssi', -999)
                if rssi > -70:
                    color = 'green'
                elif rssi > -85:
                    color = 'orange'
                else:
                    color = 'red'
                
                popup_text = f"Cell Data<br>RSSI: {rssi}dBm<br>RSRP: {data.get('rsrp', 'N/A')}dBm<br>Cell ID: {data.get('cell_id', 'N/A')}"
                
                folium.CircleMarker(
                    [data['latitude'], data['longitude']],
                    radius=5,
                    popup=popup_text,
                    color=color,
                    fill=True,
                    fillColor=color,
                    fillOpacity=0.7
                ).add_to(m)
        
        # Save to HTML and load in web view
        html_data = m._repr_html_()
        self.web_view.setHtml(html_data)
    
    def center_on_drone(self):
        """Center map on current drone position"""
        self.update_map()
    
    def clear_trail(self):
        """Clear drone trail"""
        self.drone_positions = []
        self.cellular_data_points = []
        self.update_map()
    
    def export_map(self):
        """Export map to HTML file"""
        if self.drone_positions or self.cellular_data_points:
            filename, _ = QFileDialog.getSaveFileName(self, "Save Map", f"mission_map_{int(time.time())}.html", "HTML files (*.html)")
            if filename:
                # Create full map
                center_lat = self.drone_positions[-1]['lat'] if self.drone_positions else 29.5994
                center_lon = self.drone_positions[-1]['lon'] if self.drone_positions else -95.6306
                
                m = folium.Map(location=[center_lat, center_lon], zoom_start=13)
                
                # Add all data
                if len(self.drone_positions) > 1:
                    trail_coords = [[pos['lat'], pos['lon']] for pos in self.drone_positions]
                    folium.PolyLine(trail_coords, color='blue', weight=3, opacity=0.7).add_to(m)
                
                for data in self.cellular_data_points:
                    if 'latitude' in data and 'longitude' in data:
                        rssi = data.get('rssi', -999)
                        color = 'green' if rssi > -70 else 'orange' if rssi > -85 else 'red'
                        folium.CircleMarker(
                            [data['latitude'], data['longitude']],
                            radius=5,
                            popup=f"RSSI: {rssi}dBm<br>Cell: {data.get('cell_id', 'N/A')}",
                            color=color,
                            fill=True,
                            fillColor=color,
                            fillOpacity=0.7
                        ).add_to(m)
                
                m.save(filename)
                QMessageBox.information(self, "Export Complete", f"Map saved to {filename}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Enhanced Mission Control - Cellular Data Logger")
        self.resize(1200, 800)
        
        # Connection state
        self.drone_connected = False
        self.drone_thread = None
        self.cellular_thread = None
        
        # Setup UI
        self.setup_ui()
        self.setup_status_bar()
        
        # Auto-update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(1000)  # Update every second

    def setup_ui(self):
        """Setup main UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        
        # Left panel - Controls
        left_panel = QWidget()
        left_panel.setMaximumWidth(350)
        left_layout = QVBoxLayout()
        
        # Connection controls
        connection_group = QGroupBox("Drone Connection")
        connection_layout = QVBoxLayout()
        
        self.connection_status_label = QLabel("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        
        # COM Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("COM Port:"))
        
        self.port_combo = QComboBox()
        self.populate_com_ports()
        port_layout.addWidget(self.port_combo)
        
        self.refresh_ports_btn = QPushButton("🔄")
        self.refresh_ports_btn.setMaximumWidth(30)
        self.refresh_ports_btn.setToolTip("Refresh COM ports")
        self.refresh_ports_btn.clicked.connect(self.populate_com_ports)
        port_layout.addWidget(self.refresh_ports_btn)
        
        # Baud rate selection
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '14400', '19200', '38400', '57600', '115200', '230400', '460800', '921600'])
        self.baud_combo.setCurrentText('57600')  # Default
        baud_layout.addWidget(self.baud_combo)
        
        self.button_connect = QPushButton("Connect To Drone")
        self.button_connect.clicked.connect(self.connect_to_drone)
        
        self.button_disconnect = QPushButton("Disconnect")
        self.button_disconnect.clicked.connect(self.disconnect_from_drone)
        self.button_disconnect.setEnabled(False)
        
        connection_layout.addWidget(self.connection_status_label)
        connection_layout.addLayout(port_layout)
        connection_layout.addLayout(baud_layout)
        connection_layout.addWidget(self.button_connect)
        connection_layout.addWidget(self.button_disconnect)
        connection_group.setLayout(connection_layout)
        left_layout.addWidget(connection_group)
        
        # Drone Position
        position_group = QGroupBox("Drone Position")
        position_layout = QVBoxLayout()
        
        self.label_position = QLabel("Position: N/A")
        self.label_position.setStyleSheet("font-size: 12px; color: #0074D9;")
        
        position_layout.addWidget(self.label_position)
        position_group.setLayout(position_layout)
        left_layout.addWidget(position_group)
        
        # Mission Control Commands
        commands_group = QGroupBox("Mission Control Commands")
        commands_layout = QVBoxLayout()
        
        self.button_start_logging = QPushButton("🚀 Start Cellular Logging")
        self.button_start_logging.clicked.connect(self.start_cellular_logging)
        self.button_start_logging.setStyleSheet("QPushButton { background-color: #28a745; color: white; font-weight: bold; padding: 8px; }")
        
        self.button_stop_logging = QPushButton("🛑 Stop Cellular Logging")
        self.button_stop_logging.clicked.connect(self.stop_cellular_logging)
        self.button_stop_logging.setStyleSheet("QPushButton { background-color: #dc3545; color: white; font-weight: bold; padding: 8px; }")
        
        self.button_dump_data = QPushButton("💾 Dump Data Now")
        self.button_dump_data.clicked.connect(self.dump_data)
        self.button_dump_data.setStyleSheet("QPushButton { background-color: #17a2b8; color: white; font-weight: bold; padding: 8px; }")
        
        commands_layout.addWidget(self.button_start_logging)
        commands_layout.addWidget(self.button_stop_logging)
        commands_layout.addWidget(self.button_dump_data)
        commands_group.setLayout(commands_layout)
        left_layout.addWidget(commands_group)
        
        left_layout.addStretch()
        left_panel.setLayout(left_layout)
        
        # Right panel - Tabs
        tab_widget = QTabWidget()
        
        # Map tab
        self.map_widget = MapWidget()
        tab_widget.addTab(self.map_widget, "🗺️ Map")
        
        # Cellular data tab
        self.cellular_widget = CellularDataWidget()
        tab_widget.addTab(self.cellular_widget, "📡 Cellular Data")
        
        # Message log tab
        message_widget = QWidget()
        message_layout = QVBoxLayout()
        self.message_log = QTextEdit()
        self.message_log.setReadOnly(True)
        self.message_log.setMaximumHeight(200)
        
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.clicked.connect(self.message_log.clear)
        
        message_layout.addWidget(QLabel("System Messages:"))
        message_layout.addWidget(self.message_log)
        message_layout.addWidget(clear_log_btn)
        message_widget.setLayout(message_layout)
        tab_widget.addTab(message_widget, "📋 Messages")
        
        # Add to main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(tab_widget)
        
        central_widget.setLayout(main_layout)

    def populate_com_ports(self):
        """Populate COM port dropdown with available ports"""
        import serial.tools.list_ports
        
        current_selection = self.port_combo.currentText() if hasattr(self, 'port_combo') else None
        
        self.port_combo.clear()
        
        # Get available COM ports
        available_ports = []
        ports = serial.tools.list_ports.comports()
        
        for port in sorted(ports):
            port_info = f"{port.device}"
            if port.description and port.description != 'n/a':
                port_info += f" ({port.description})"
            available_ports.append((port.device, port_info))
        
        # Add ports to combo box
        if available_ports:
            for device, info in available_ports:
                self.port_combo.addItem(info, device)  # Display name, actual device stored as data
        else:
            self.port_combo.addItem("No COM ports found", None)
        
        # Add manual entry option
        self.port_combo.addItem("Manual Entry...", "MANUAL")
        
        # Restore previous selection if it still exists
        if current_selection:
            index = self.port_combo.findText(current_selection)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
        
        self.statusBar().showMessage(f"Found {len(available_ports)} COM ports")

    def get_selected_port(self):
        """Get the currently selected COM port"""
        if self.port_combo.currentData() == "MANUAL":
            # Show input dialog for manual entry
            from PyQt5.QtWidgets import QInputDialog
            port, ok = QInputDialog.getText(self, 'Manual COM Port Entry', 
                                          'Enter COM port (e.g., COM4, /dev/ttyUSB0):')
            if ok and port:
                return port.strip()
            else:
                return None
        elif self.port_combo.currentData() is None:
            self.show_error("No COM port selected or available")
            return None
        else:
            return self.port_combo.currentData()

    def get_selected_baud(self):
        """Get the currently selected baud rate"""
        try:
            return int(self.baud_combo.currentText())
        except ValueError:
            return 57600  # Default fallback

    def setup_status_bar(self):
        """Setup status bar"""
        self.statusBar().showMessage("Ready - Connect to drone to begin")

    def connect_to_drone(self):
        """Connect to drone"""
        # Get selected port and baud rate
        selected_port = self.get_selected_port()
        if not selected_port:
            return
            
        selected_baud = self.get_selected_baud()
        
        print(f"Connecting to drone on {selected_port} at {selected_baud} baud...")
        
        if self.drone_thread is not None:
            self.disconnect_from_drone()
        
        # Create new drone thread with selected parameters
        self.drone_thread = DronePositionThread(port=selected_port, baud=selected_baud)
        self.drone_thread.position_update.connect(self.update_drone_position)
        self.drone_thread.connection_status.connect(self.update_connection_status)
        self.drone_thread.start()
        
        self.button_connect.setEnabled(False)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self.refresh_ports_btn.setEnabled(False)
        
        self.connection_status_label.setText(f"Status: Connecting to {selected_port}...")
        self.connection_status_label.setStyleSheet("color: orange; font-weight: bold;")
        self.statusBar().showMessage(f"Connecting to {selected_port} at {selected_baud} baud...")

    def disconnect_from_drone(self):
        """Disconnect from drone"""
        if self.drone_thread is not None:
            self.drone_thread.stop()
            self.drone_thread = None
        
        if self.cellular_thread is not None:
            self.cellular_thread.stop()
            self.cellular_thread = None
            
        self.drone_connected = False
        self.button_connect.setEnabled(True)
        self.button_disconnect.setEnabled(False)
        
        # Re-enable port selection controls
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
        self.refresh_ports_btn.setEnabled(True)
        
        self.connection_status_label.setText("Status: Disconnected")
        self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        self.statusBar().showMessage("Disconnected")

    def update_connection_status(self, connected, message):
        """Update connection status"""
        self.drone_connected = connected
        
        if connected:
            self.connection_status_label.setText("Status: Connected")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
            self.button_connect.setEnabled(False)
            self.button_disconnect.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.refresh_ports_btn.setEnabled(False)
            
            # Start cellular data thread
            if self.cellular_thread is None:
                self.cellular_thread = CellularDataThread(self.drone_thread.master)
                self.cellular_thread.cellular_data_received.connect(self.handle_cellular_data)
                self.cellular_thread.mission_status_update.connect(self.handle_mission_status)
                self.cellular_thread.raw_message_received.connect(self.handle_raw_message)
                self.cellular_thread.start()
                
            self.statusBar().showMessage(f"Connected: {message}")
            self.log_message(f"✅ {message}")
            
        else:
            self.connection_status_label.setText("Status: Connection Failed")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.button_connect.setEnabled(True)
            self.button_disconnect.setEnabled(False)
            self.port_combo.setEnabled(True)
            self.baud_combo.setEnabled(True)
            self.refresh_ports_btn.setEnabled(True)
            self.statusBar().showMessage(f"Connection failed: {message}")
            self.log_message(f"❌ Connection failed: {message}")

    def update_drone_position(self, lat, lon, alt):
        """Update drone position display"""
        self.label_position.setText(f"Lat: {lat:.7f}\nLon: {lon:.7f}\nAlt: {alt:.2f} m")
        self.map_widget.update_drone_position(lat, lon, alt)
        self.statusBar().showMessage(f"Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")

    def handle_cellular_data(self, data):
        """Handle received cellular data"""
        self.cellular_widget.update_cellular_data(data)
        self.map_widget.add_cellular_data_point(data)
        
        # Log significant cellular events
        if 'rssi' in data and data['rssi'] < -100:
            self.log_message(f"⚠️ Poor signal: RSSI {data['rssi']}dBm")
        elif 'cell_id' in data:
            self.log_message(f"📡 Cell data: ID {data['cell_id']}, RSSI {data.get('rssi', 'N/A')}dBm")

    def handle_mission_status(self, status_type, data):
        """Handle mission status updates"""
        self.cellular_widget.update_mission_status(status_type, data)
        
        if status_type == 'mission_started':
            self.log_message(f"🚀 Mission started: ID {data['mission_id']}")
        elif status_type == 'mission_stopped':
            self.log_message(f"🏁 Mission completed: {data['data_points']} data points collected")

    def handle_raw_message(self, message_type, content):
        """Handle raw messages from drone"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_message(f"[{timestamp}] {message_type}: {content}")

    def log_message(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        formatted_message = f"[{timestamp}] {message}"
        self.message_log.append(formatted_message)
        
        # Keep log manageable
        if self.message_log.document().lineCount() > 1000:
            cursor = self.message_log.textCursor()
            cursor.movePosition(cursor.Start)
            for _ in range(100):  # Remove first 100 lines
                cursor.select(cursor.LineUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

    def start_cellular_logging(self):
        """Send start cellular logging command"""
        if self.is_drone_connected():
            try:
                self.drone_thread.master.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,
                    b'START_CELL_LOG'
                )
                self.log_message("📤 Sent: START_CELL_LOG command")
                self.statusBar().showMessage("Cellular logging start command sent")
            except Exception as e:
                self.show_error(f"Failed to send start command: {e}")
        else:
            self.show_error("Drone not connected")

    def stop_cellular_logging(self):
        """Send stop cellular logging command"""
        if self.is_drone_connected():
            try:
                self.drone_thread.master.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,
                    b'STOP_CELL_LOG'
                )
                self.log_message("📤 Sent: STOP_CELL_LOG command")
                self.statusBar().showMessage("Cellular logging stop command sent")
            except Exception as e:
                self.show_error(f"Failed to send stop command: {e}")
        else:
            self.show_error("Drone not connected")

    def dump_data(self):
        """Send dump data command"""
        if self.is_drone_connected():
            try:
                self.drone_thread.master.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,
                    b'DUMP_CELL_DATA'
                )
                self.log_message("📤 Sent: DUMP_CELL_DATA command")
                self.statusBar().showMessage("Data dump command sent")
            except Exception as e:
                self.show_error(f"Failed to send dump command: {e}")
        else:
            self.show_error("Drone not connected")

    def is_drone_connected(self):
        """Check if drone is connected"""
        return (
            self.drone_connected and
            self.drone_thread is not None and
            self.drone_thread.master is not None and
            hasattr(self.drone_thread.master, 'target_system')
        )

    def show_error(self, message):
        """Show error dialog"""
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Critical)
        msg_box.setWindowTitle("Error")
        msg_box.setText(message)
        msg_box.exec_()
        self.log_message(f"❌ Error: {message}")

    def show_info(self, message):
        """Show info dialog"""
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setWindowTitle("Information")
        msg_box.setText(message)
        msg_box.exec_()

    def update_display(self):
        """Periodic display updates"""
        # Update button states based on connection
        commands_enabled = self.is_drone_connected()
        self.button_start_logging.setEnabled(commands_enabled)
        self.button_stop_logging.setEnabled(commands_enabled)
        self.button_dump_data.setEnabled(commands_enabled)

    def closeEvent(self, event):
        """Handle application close"""
        self.log_message("🔄 Shutting down...")
        
        if self.drone_thread is not None:
            self.drone_thread.stop()
        if self.cellular_thread is not None:
            self.cellular_thread.stop()
            
        # Wait a moment for threads to stop
        time.sleep(1)
        event.accept()


# Import required for MAVLink in threads
from pymavlink import mavutil


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Apply dark theme
    dark_palette = app.palette()
    dark_palette.setColor(dark_palette.Window, QColor(53, 53, 53))
    dark_palette.setColor(dark_palette.WindowText, QColor(255, 255, 255))
    dark_palette.setColor(dark_palette.Base, QColor(25, 25, 25))
    dark_palette.setColor(dark_palette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(dark_palette.ToolTipBase, QColor(0, 0, 0))
    dark_palette.setColor(dark_palette.ToolTipText, QColor(255, 255, 255))
    dark_palette.setColor(dark_palette.Text, QColor(255, 255, 255))
    dark_palette.setColor(dark_palette.Button, QColor(53, 53, 53))
    dark_palette.setColor(dark_palette.ButtonText, QColor(255, 255, 255))
    dark_palette.setColor(dark_palette.BrightText, QColor(255, 0, 0))
    dark_palette.setColor(dark_palette.Link, QColor(42, 130, 218))
    dark_palette.setColor(dark_palette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(dark_palette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(dark_palette)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())