#!/bin/bash

# Push qurt image to voxl2
adb push build/modalai_voxl2_qurt/platforms/qurt/libpx4.so /usr/lib/rfsa/adsp

# Push posix image to voxl2
adb push build/modalai_voxl2_default/bin/px4 /usr/bin

# Push scripts to voxl2
adb push build/modalai_voxl2_default/bin/px4-alias.sh /usr/bin
adb push boards/modalai/voxl2/voxl-px4 /usr/bin
adb shell chmod a+x /usr/bin/px4-alias.sh
adb shell chmod a+x /usr/bin/voxl-px4

# Push configuration file
adb shell mkdir -p /etc/modalai
adb push boards/modalai/voxl2/voxl-px4.config /etc/modalai

adb shell sync
