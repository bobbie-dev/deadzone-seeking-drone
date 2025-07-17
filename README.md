# 📡 Cellular Signal Quality Logger & Visualizer

This project logs cellular module signal quality data while in motion (e.g., on a drone) and visualizes it on a map using Python. It is useful for testing coverage, handoff behavior, and connectivity in mobile scenarios.

---

## 🧰 Features

- Logs signal quality (RSSI, RSRP, RSRQ, SINR) from cellular modules (e.g., SIM7600, SIM8200)
- Logs GPS data (lat, lon, alt, timestamp)
- Saves data to CSV or NetCDF format
- Visualizes signal strength across geospatial locations using `matplotlib` and `Basemap`

---

<<<<<<< HEAD
## Groundbase Setup

- Need to have python 3 & pip installed and working
- ```pip install pyqt5 PyQtWebEngine shapely```

---

## SITL Setup

- Install WSL through system elevated command line ```wsl --install```
- Clone https://github.com/PX4/PX4-Autopilot.git
- Then run these in WSL ```
			cd PX4-Autopilot
			bash ./Tools/setup/ubuntu.sh   # or macOS equivalent
		```

