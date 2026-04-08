@echo off
REM Script để fix lỗi checksum mismatch bằng cách clean build và flash lại

echo ========================================
echo Fixing Checksum Mismatch
echo ========================================
echo.

REM Clean build
echo [1/3] Cleaning build directory...
idf.py fullclean
if %errorlevel% neq 0 (
    echo ERROR: Clean failed!
    pause
    exit /b 1
)

REM Build lại project
echo.
echo [2/3] Building project...
idf.py build
if %errorlevel% neq 0 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

REM Flash lại toàn bộ firmware
echo.
echo [3/3] Flashing firmware to device...
echo Please make sure your ESP32 device is connected!
echo.
idf.py flash
if %errorlevel% neq 0 (
    echo ERROR: Flash failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo SUCCESS! Checksum mismatch should be fixed.
echo ========================================
pause










































































