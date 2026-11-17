# `ramabot_mapping` — Standalone 2D LiDAR Mapping Package

Paket ROS 2 Humble mandiri untuk akuisisi peta 2D (`SLAM Toolbox` + `RPLidar A2M8`) menggunakan robot differential drive berkerangka terbuka sebelum dimuat ke **RAMABOT V3** (`Nav2` / `AMCL`).

## 1. Pengujian Offline di WSL2 (Tanpa Hardware)
```bash
cd ~/ramabot_ws
colcon build --packages-select ramabot_mapping --symlink-install
source install/setup.bash

ros2 launch ramabot_mapping mapping_wsl_test.launch.py