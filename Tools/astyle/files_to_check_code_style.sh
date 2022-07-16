#!/usr/bin/env bash
set -eu

PATTERN="-e ."

if [ $# -gt 0 ]; then
    PATTERN="$1"
fi

exec find boards msg src platforms test \
    -not -path "msg/templates/urtps/*" \
    -not -path "platforms/nuttx/NuttX/*" \
    -not -path "platforms/qurt/dspal/*" \
    -not -path "src/drivers/gps/devices/*" \
    -not -path "src/drivers/uavcan/libuavcan/*" \
    -not -path "src/drivers/uavcan/uavcan_drivers/kinetis/driver/include/uavcan_kinetis/*" \
    -not -path "src/drivers/cyphal/libcanard/*" \
    -not -path "src/lib/crypto/monocypher/*" \
    -not -path "src/lib/events/libevents/*" \
    -not -path "src/lib/parameters/uthash/*" \
    -not -path "src/modules/ekf2/EKF/*" \
    -not -path "src/modules/gyro_fft/CMSIS_5/*" \
    -not -path "src/modules/mavlink/mavlink/*" \
    -not -path "src/modules/micrortps_bridge/micro-CDR/*" \
    -not -path "src/modules/micrortps_bridge/microRTPS_client/*" \
    -not -path "test/mavsdk_tests/catch2/*" \
    -not -path "src/lib/crypto/monocypher/*" \
    -not -path "src/lib/crypto/libtomcrypt/*" \
    -not -path "src/lib/crypto/libtommath/*" \
    -not -path "src/modules/microdds_client/Micro-XRCE-DDS-Client/*" \
    -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) | grep $PATTERN
