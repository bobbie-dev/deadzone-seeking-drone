import sys
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QVBoxLayout, QWidget,
    QPushButton, QSpinBox, QLabel, QHBoxLayout
)
from PyQt5.QtCore import QThread, pyqtSignal


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
            print(f"Connected. Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
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


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Control UI")
        self.resize(600, 300)

        # Controls
        self.label_position = QLabel("Drone Position: N/A")
        self.label_position.setStyleSheet("font-size: 16px; color: #0074D9;")


        self.button_connect = QPushButton("Connect To Drone")
        self.button_connect.clicked.connect(self.connect_to_drone)


        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label_position)
        layout.addSpacing(10)
        layout.addWidget(self.button_connect)
        layout.addStretch()

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.waypoints = []
        self.drone_thread = None

    def connect_to_drone(self):
        print("Connecting to drone...")
        if self.drone_thread is not None:
            self.drone_thread.stop()
            self.drone_thread = None
        self.drone_thread = DronePositionThread(port='COM4', baud=57600)
        self.drone_thread.position_update.connect(self.update_drone_position)
        self.drone_thread.start()

    def update_drone_position(self, lat, lon, alt):
        self.label_position.setText(f"Drone Position: Lat {lat:.7f}, Lon {lon:.7f}, Alt {alt:.2f} m")

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
        print("Send to drone clicked (placeholder)")

    def closeEvent(self, event):
        if self.drone_thread is not None:
            self.drone_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
