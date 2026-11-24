# Development Log — `ramabot_mapping` (Standalone 2D Mapping Package)
- **Tanggal:** 18 November 2026
- **Environment Pengembangan:** WSL2 Ubuntu 22.04 LTS (`kizuna@raka01`) | ROS 2 Humble | `rmw_fastrtps_cpp`
- **Target Deployment:** NVIDIA Jetson Xavier NX + ESP32 DevKit Base Controller + RPLidar A2M8
- **Repository:** `https://github.com/KizunaK15/ramabot_mapping.git`

## 1. Latar Belakang Arsitektur
Pemisahan robot akuisisi peta dilakukan karena struktur kerangka luar **RAMABOT V3** menghalangi bidang pandang 360° **RPLidar A2M8** saat proses pemetaan ruangan. Peta 2D (`.yaml` + `.pgm`) direkam menggunakan robot differential drive berkerangka terbuka melalui `ramabot_mapping`, kemudian hasilnya dimuat ke stack navigasi otonom (`map_server` + `AMCL` + `Nav2`) pada **RAMABOT V3**.

## 2. Perubahan Teknis & Perbaikan Firmware (`firmware/mapping_controller/`)
1. **Integrasi Interrupt Encoder (`mapping_controller.ino`):** Menambahkan ISR kuadratur (`ENC_L_A/B`, `ENC_R_A/B`) pada pin input-only ESP32 (placeholder GPIO `34, 35, 36, 39`) untuk menghitung pulsa roda secara real-time.
2. **Telemetri Serial 20 Hz (`115200` bps):** Mengirimkan string terformat `ENCL:<val>,ENCR:<val>,YAW:<deg>,PWML:<val>,PWMR:<val>\n` melalui USB Serial ke Jetson Xavier NX tanpa mengganggu kendali manual stik PS3 (`Bluepad32`).
3. **Deadzone Analog Stick PS3 (`±25`):** Mengatasi *jitter* tuas analog saat posisi netral agar timer `toInPID` tidak ter-reset dan *Outer-Loop Gyro Heading-Hold PID* (MPU6050) tetap aktif saat robot berjalan lurus.
4. **Perbaikan Bug Menu OLED (`menu_utama.ino`):** Menambahkan perintah `break;` pada akhir blok `case 4:` (*Reset Parameter?*) untuk mencegah *fall-through* ke `case 3:` (*Reset connection?*) yang sebelumnya menghapus MAC address stik PS3 dan me-restart ESP32.

## 3. Implementasi Paket ROS 2 (`ramabot_mapping`)
1. **`mapping_odom_bridge.py`:** Mengonversi delta pulsa encoder serial menjadi topik `/odom` (`nav_msgs/Odometry`) dan menyiarkan TF `odom -> base_footprint`. Dilengkapi pengaturan `ser.dtr = False` dan `ser.rts = False` untuk mencegah *auto-reset* board ESP32 saat node ROS 2 dijalankan, serta mekanisme *auto-reconnect* serial.
2. **`wsl_serial_mock.py` & `mapping_wsl_test.launch.py`:** Simulator *raycasting* LiDAR 360° dan odometri virtual untuk memvalidasi rantai TF (`map -> odom -> base_footprint -> laser_frame`), `async_slam_toolbox_node` (Google Ceres Solver), dan `foxglove_bridge` (port `8765`) secara offline di WSL2.
3. **`setup_udev_jetson.sh` & `save_map.sh`:** Skrip otomasi *udev rules* berbasis `KERNELS` USB port untuk mengunci symlink `/dev/rplidar` dan `/dev/esp32_map` agar tidak tertukar saat *booting* di Jetson Xavier NX.