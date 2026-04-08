#!/bin/bash
# Script để fix lỗi checksum mismatch bằng cách clean build và flash lại

echo "========================================"
echo "Fixing Checksum Mismatch"
echo "========================================"
echo ""

# Clean build
echo "[1/3] Cleaning build directory..."
idf.py fullclean
if [ $? -ne 0 ]; then
    echo "ERROR: Clean failed!"
    exit 1
fi

# Build lại project
echo ""
echo "[2/3] Building project..."
idf.py build
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed!"
    exit 1
fi

# Flash lại toàn bộ firmware
echo ""
echo "[3/3] Flashing firmware to device..."
echo "Please make sure your ESP32 device is connected!"
echo ""
idf.py flash
if [ $? -ne 0 ]; then
    echo "ERROR: Flash failed!"
    exit 1
fi

echo ""
echo "========================================"
echo "SUCCESS! Checksum mismatch should be fixed."
echo "========================================"










































































