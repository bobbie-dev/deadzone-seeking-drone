import sys,json,os
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QVBoxLayout, QWidget, QPushButton, QSpinBox, QLabel, QHBoxLayout, QSpacerItem, QSizePolicy
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import QObject, pyqtSlot, QUrl
from shapely.geometry import Polygon

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

        self.browser.load(QUrl.fromLocalFile(os.getcwd()+"/map.html".replace("\\", "/")))

        # GUI Controls
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
        right_panel.addWidget(self.button_send_to_drone)
        right_panel.addStretch()

        # Add map and panel to main layout
        main_layout.addWidget(self.browser)
        main_layout.addLayout(right_panel)

        # Set layout to container
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)


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
    
    def send_to_drone(self):
        pass


if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_stylesheet(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
