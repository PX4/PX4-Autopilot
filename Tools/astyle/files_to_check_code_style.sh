#!/usr/bin/env bash
set -eu

PATTERN="-e ."

if [ $# -gt 0 ]; then
    PATTERN="$1"
fi

exec find boards msg src platforms test \
    -path platforms/nuttx/NuttX -prune -o \
    -path platforms/qurt/dspal -prune -o \
    -path src/drivers/ins/vectornav/libvnc -prune -o \
    -path src/drivers/uavcan/libdronecan -prune -o \
    -path src/drivers/uavcan/uavcan_drivers/kinetis/driver/include/uavcan_kinetis -prune -o \
    -path src/drivers/cyphal/libcanard -prune -o \
    -path src/lib/crypto/monocypher -prune -o \
    -path src/lib/events/libevents -prune -o \
    -path src/lib/parameters/uthash -prune -o \
    -path src/lib/wind_estimator/python/generated -prune -o \
    -path src/modules/ekf2/EKF/python/ekf_derivation/generated -prune -o \
    -path src/modules/ekf2/EKF/yaw_estimator/derivation/generated -prune -o \
    -path src/modules/gyro_fft/CMSIS_5 -prune -o \
    -path src/modules/mavlink/mavlink -prune -o \
    -path test/mavsdk_tests/catch2 -prune -o \
    -path src/lib/crypto/monocypher -prune -o \
    -path src/lib/crypto/libtomcrypt -prune -o \
    -path src/lib/crypto/libtommath -prune -o \
    -path src/lib/heatshrink/heatshrink -prune -o \
    -path src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client -prune -o \
    -path src/lib/cdrstream/cyclonedds -prune -o \
    -path src/lib/cdrstream/rosidl -prune -o \
    -path src/modules/zenoh/zenoh-pico -prune -o \
    -path boards/modalai/voxl2/libfc-sensor-api -prune -o \
    -path src/drivers/actuators/vertiq_io/iq-module-communication-cpp -prune -o \
    \( -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) -print \) | grep $PATTERN
