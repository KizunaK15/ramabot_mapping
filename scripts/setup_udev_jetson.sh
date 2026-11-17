#!/usr/bin/env bash
set -e

echo "=== Deteksi Perangkat USB Serial Terpasang ==="
for dev in /dev/ttyUSB*; do
  if [ -e "$dev" ]; then
    SERIAL=$(udevadm info -a -n "$dev" | grep -m 1 'ATTRS{serial}' | cut -d'"' -f2)
    VENDOR=$(udevadm info -a -n "$dev" | grep -m 1 'ATTRS{idVendor}' | cut -d'"' -f2)
    PRODUCT=$(udevadm info -a -n "$dev" | grep -m 1 'ATTRS{idProduct}' | cut -d'"' -f2)
    KERN=$(udevadm info -a -n "$dev" | grep -m 1 'KERNELS==' | cut -d'"' -f2)
    echo "Device: $dev | VID:PID=$VENDOR:$PRODUCT | Serial=$SERIAL | USB_Port=$KERN"
  fi
done

echo ""
echo "Masukkan USB_Port (contoh: 1-2.1) untuk RPLidar A2M8:"
read -r LIDAR_PORT_PATH
echo "Masukkan USB_Port (contoh: 1-2.2) untuk Modul Base ESP32:"
read -r ESP_PORT_PATH

RULE_FILE="/etc/udev/rules.d/99-ramabot-mapping.rules"
sudo bash -c "cat > $RULE_FILE" << EOL
SUBSYSTEM=="tty", KERNELS=="$LIDAR_PORT_PATH", MODE:="0666", SYMLINK+="rplidar"
SUBSYSTEM=="tty", KERNELS=="$ESP_PORT_PATH", MODE:="0666", SYMLINK+="esp32_map"
EOL

sudo usermod -aG dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Selesai! Symlink /dev/rplidar dan /dev/esp32_map aktif."
ls -l /dev/rplidar /dev/esp32_map 2>/dev/null || true
