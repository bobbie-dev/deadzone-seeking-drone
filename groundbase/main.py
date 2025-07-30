import sys, time, struct
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QVBoxLayout, QWidget,
    QPushButton, QSpinBox, QLabel, QHBoxLayout, QMessageBox
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
        if self.master:
            self.master.close()
        self.wait()


class DroneDataThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, master):
        super().__init__()
        self.master = master
        self._running = True

    def run(self):
        while self._running:
            try:
                # Fixed: Use self.master instead of self.DronePositionThread.master
                msg = self.master.recv_match(type='CUSTOM_MSG', blocking=True, timeout=2)
                if msg:
                    data = struct.unpack('!iiihhhhhhhI', msg)
                    self.data_received.emit(data)
            except Exception as e:
                print(f"Error receiving data: {e}")
                continue

    def stop(self):
        self._running = False
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Control UI")
        self.resize(600, 300)

        # Drone Position
        self.label_position = QLabel("Drone Position: N/A")
        self.label_position.setStyleSheet("font-size: 16px; color: #0074D9;")

        # Connect to Drone
        self.button_connect = QPushButton("Connect To Drone")
        self.button_connect.clicked.connect(self.connect_to_drone)

        # Data Dump
        self.button_data_dump = QPushButton("Dump Data")
        self.button_data_dump.clicked.connect(self.dump_data)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label_position)
        layout.addSpacing(10)
        layout.addWidget(self.button_connect)
        layout.addSpacing(10)
        layout.addWidget(self.button_data_dump)
        layout.addStretch()

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.waypoints = []
        self.drone_thread = None
        self.data_thread = None  # Added missing data thread reference

    def connect_to_drone(self):
        print("Connecting to drone...")
        if self.drone_thread is not None:
            self.drone_thread.stop()
            self.drone_thread = None
        if self.data_thread is not None:
            self.data_thread.stop()
            self.data_thread = None
            
        self.drone_thread = DronePositionThread(port='COM4', baud=57600)
        self.drone_thread.position_update.connect(self.update_drone_position)
        self.drone_thread.start()

    def is_drone_connected(self):
        return (
            self.drone_thread is not None and
            self.drone_thread.master is not None and
            getattr(self.drone_thread.master, 'target_system', None) is not None
        )

    def update_drone_position(self, lat, lon, alt):
        self.label_position.setText(f"Drone Position: Lat {lat:.7f}, Lon {lon:.7f}, Alt {alt:.2f} m")

    def dump_data(self):
        if self.is_drone_connected():
            # Fixed: Use self.drone_thread.master instead of self.DronePositionThread.master
            self.drone_thread.master.mav.statustext_send(
                self.drone_thread.master.target_system, 
                self.drone_thread.master.target_component,
                b'DUMP_DATA'
            )
        else:
            ed = QMessageBox()
            ed.setIcon(QMessageBox.Critical)
            ed.setText("Error")
            ed.setInformativeText('Drone Not Connected')
            ed.setWindowTitle("Error")
            ed.exec_()


    def closeEvent(self, event):
        if self.drone_thread is not None:
            self.drone_thread.stop()
        if self.data_thread is not None:
            self.data_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())