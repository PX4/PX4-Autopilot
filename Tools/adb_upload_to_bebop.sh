#!/bin/bash

if [ -z ${BEBOP_IP+x} ]; then 
  ip=192.168.42.1
  echo "\$BEBOP_IP is not set (use default: $ip)"
else 
  ip=$BEBOP_IP
  echo "\$BEBOP_IP is set to $ip"
fi
port=9050

echo "Connecting to bebop: $ip:$port"

# adb returns also 0 as exit status if the connection fails
adb_return=$(adb connect $ip:$port)
adb_status=$(echo $adb_return | cut -f 1 -d " ")

if [[ $adb_status == "unable" ]]; then
  
  echo ""
  echo "Connection with Parrot Bebop could not be established:"
  echo "  Make sure you are connected with the Bebop's WiFi and"
  echo "  enable access to the board by pressing the power button 4 times."
  echo ""
  exit 50

fi

echo "Connection successfully established"

sleep 1

adb shell mount -o remount,rw /
adb shell touch /home/root/parameters
adb shell mkdir -p /data/ftp/internal_000/fs/microsd

# kill PX4 if it is already running from autostart
restart_px4=false
adb_return=$(adb shell killall -KILL px4)
if [[ $adb_return == "" ]]; then
    echo "Killed running PX4 process"
    restart_px4=true
fi

${PX4_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-strip \
  -R .comment -R .gnu.version \
  ../build_posix_bebop_default/src/firmware/posix/px4

../Tools/adb_upload.sh $@

# restart the process after uploading
if [ "$restart_px4" = true ]; then
    echo "Restarting PX4 process"
    adb shell /etc/init.d/rcS_mode_default 2>/dev/null 1>/dev/null &
fi

echo "Disconnecting from Bebop"
adb disconnect
