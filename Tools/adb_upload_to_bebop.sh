#!/bin/bash

ip=192.168.42.1
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

../Tools/adb_upload.sh $@

echo "Disconnecting from bebop"
adb disconnect
