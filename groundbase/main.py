import sys,json,os,time
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QVBoxLayout, QWidget, QPushButton, QSpinBox, QLabel, QHBoxLayout, QSpacerItem, QSizePolicy
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import QObject, pyqtSlot, QUrl, QThread, pyqtSignal
from shapely.geometry import Polygon
from pymavlink import mavutil



class Bridge(QObject):
    def __init__(self, waypoint_callback):
        super().__init__()
        self.num_waypoints = 10  # default
        self.waypoint_callback = waypoint_callback

    @pyqtSlot(str)
    def sendPolygon(self, coords_json):
        coords = json.loads(coords_json)  # [[lat, lon], ...]
        print("Received polygon with", len(coords), "points")

        # Convert to (lon, lat) for Shapely
        polygon = Polygon([(lon, lat) for lat, lon in coords])
        perimeter = polygon.exterior
        length = perimeter.length

        waypoints = []
        for i in range(self.num_waypoints):
            pt = perimeter.interpolate(length * i / self.num_waypoints)
            waypoints.append((round(pt.y, 7), round(pt.x, 7)))  # (lat, lon)

        waypoints.append(waypoints[0])  # Return to start
        print(f"\nGenerated {len(waypoints)} waypoints (including return to start):")
        for i, wp in enumerate(waypoints):
            print(f"{i+1}: {wp}")

        self.waypoint_callback(waypoints)

class DronePositionThread(QThread):
    position_update = pyqtSignal(float, float, float)  # lat, lon, alt

    def __init__(self, port='COM4', baud=57600):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self.master = None

    def run(self):
        from pymavlink import mavutil
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.master.wait_heartbeat(timeout=10)
        except Exception as e:
            print(f"Drone connection error: {e}")
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
                print(f"Error receiving drone position: {e}")
                continue

    def stop(self):
        self._running = False
        self.wait()

def apply_stylesheet(app, path="style.qss"):
    try:
        with open(path, "r") as f:
            style = f.read()
            app.setStyleSheet(style)
    except FileNotFoundError:
        print(f"Stylesheet {path} not found. Using default.")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Grid Mission Planner")
        self.resize(1200, 800)

        self.browser = QWebEngineView()
        self.channel = QWebChannel()
        self.bridge = Bridge(self.handle_waypoints)
        self.channel.registerObject("pyObj", self.bridge)
        self.browser.page().setWebChannel(self.channel)

        self.browser.load(QUrl.fromLocalFile(os.path.join(os.getcwd(), "groundbase", "map.html").replace("\\", "/")))
        # GUI Controls

        # Connect to Drone
        self.button_connect_to_drone = QPushButton("Connect To Drone")
        self.button_connect_to_drone.clicked.connect(self.connect_to_drone)


        # Waypoint
        self.waypoints = []
        self.button_export = QPushButton("Export .waypoints File")
        self.button_export.clicked.connect(self.export_waypoints)


        # Send to Drone
        self.button_send_to_drone = QPushButton("Send To Drone")
        self.button_send_to_drone.clicked.connect(self.send_to_drone)


        self.label = QLabel("Waypoints:")
        self.spinner = QSpinBox()
        self.spinner.setRange(3, 100)
        self.spinner.setValue(10)
        self.spinner.valueChanged.connect(self.set_waypoint_count)

        # Add drone position label
        self.label_position = QLabel("Drone Position: N/A")
        self.label_position.setStyleSheet("font-size: 16px; color: #0074D9;")

        # Create layout containers
        main_layout = QHBoxLayout()
        right_panel = QVBoxLayout()

        # Make the map fill most of the screen
        self.browser.setMinimumWidth(900)

        # Make buttons bigger
        self.button_export.setFixedHeight(40)
        self.spinner.setFixedHeight(30)
        self.label.setStyleSheet("font-size: 16px;")

        self.button_export.setStyleSheet("font-size: 16px; padding: 6px;")

        # Add widgets to right panel
        right_panel.addWidget(self.label)
        right_panel.addWidget(self.spinner)
        right_panel.addSpacing(20)
        right_panel.addWidget(self.button_export)
        right_panel.addWidget(self.button_connect_to_drone)
        right_panel.addWidget(self.label_position)

        # Add stretch to push button to bottom
        right_panel.addStretch()

        # Create a horizontal layout to hold the button aligned right
        button_layout = QHBoxLayout()
        button_layout.addStretch()  # Push button to the right

        # Style the "Send To Drone" button red
        self.button_send_to_drone.setStyleSheet("""
            background-color: #d9534f;
            color: white;
            font-size: 16px;
            padding: 6px;
            border: none;
            border-radius: 4px;
        """)

        button_layout.addWidget(self.button_send_to_drone)

        # Add the button layout to the right panel
        right_panel.addLayout(button_layout)

        # Add map and panel to main layout
        main_layout.addWidget(self.browser)
        main_layout.addLayout(right_panel)

        # Set layout to container
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Drone position thread
        self.drone_thread = None


    def set_waypoint_count(self, value):
        self.bridge.num_waypoints = value

    def handle_waypoints(self, waypoints):
        self.waypoints = waypoints

    def export_waypoints(self):
        if not self.waypoints:
            print("No waypoints to export.")
            return

        filename, _ = QFileDialog.getSaveFileName(self, "Save Mission File", "", "Pixhawk Waypoints (*.waypoints)")
        if filename:
            if not filename.endswith(".waypoints"):
                filename += ".waypoints"
            with open(filename, "w") as f:
                f.write("QGC WPL 110\n")
                for i, (lat, lon) in enumerate(self.waypoints):
                    f.write(f"{i}\t0\t16\t16\t0\t0\t0\t0\t{lat}\t{lon}\t20.0\t1\n")
            print(f"Saved to {filename}")
    
    def update_drone_position(self, lat, lon, alt):
        self.label_position.setText(f"Drone Position: Lat {lat:.7f}, Lon {lon:.7f}, Alt {alt:.2f} m")

    def connect_to_drone(self):# Mavlink Setup 
        print("Connecting to drone...")
        if self.drone_thread is not None:
            self.drone_thread.stop()
            self.drone_thread = None
        self.drone_thread = DronePositionThread(port='COM4', baud=57600)
        self.drone_thread.position_update.connect(self.update_drone_position)
        self.drone_thread.start()


    def send_to_drone(self):
        pass

    def closeEvent(self, event):
        if self.drone_thread is not None:
            self.drone_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_stylesheet(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
