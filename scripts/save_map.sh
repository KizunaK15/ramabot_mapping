#!/usr/bin/env bash
set -e

MAP_NAME="${1:-hospital_map}"
OUT_DIR="$HOME/ramabot_ws/src/ramabot_mapping/maps"
mkdir -p "$OUT_DIR"

echo "Menyimpan peta ke $OUT_DIR/$MAP_NAME (.yaml & .pgm)..."
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$MAP_NAME"
echo "Berhasil! File siap dimuat ke Nav2 AMCL / map_server RAMABOT V3."
