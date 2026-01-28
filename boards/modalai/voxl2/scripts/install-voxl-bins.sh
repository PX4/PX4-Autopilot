#!/bin/bash

# Push slpi image to voxl2
adb push build/modalai_voxl2-slpi_default/platforms/qurt/libpx4.so /usr/lib/rfsa/adsp

# Push apps processor image to voxl2
adb push build/modalai_voxl2_default/bin/px4 /usr/bin

adb shell sync
