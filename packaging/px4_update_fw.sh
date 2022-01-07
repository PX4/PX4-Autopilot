#!/bin/bash

port=$(/opt/px4fwupdater/detect_ttyS.sh)

if [ -z $port ]
then
  echo "Unable to detect serial port. Exiting.."
  exit 1
fi

systemctl is-active agent_protocol_splitter.service &> /dev/null
if [ $? -eq 0 ]
then
  echo "Stopping agent_protocol_splitter before flashing"
  systemctl stop agent_protocol_splitter.service
  echo "Start flashing.. port /dev/ttyS${port}"
  /opt/px4fwupdater/px_uploader.py --port /dev/ttyS${port} --baud-bootloader 115200 --baud-flightstack 1000000 /opt/px4fwupdater/px4_fmu-v5_ssrc.px4
  echo "Start agent_protocol_splitter"
  systemctl start agent_protocol_splitter.service
else
  echo "Start flashing.. port /dev/ttyS{port}"
  /opt/px4fwupdater/px_uploader.py --port /dev/ttyS${port} --baud-bootloader 115200 --baud-flightstack 1000000 /opt/px4fwupdater/px4_fmu-v5_ssrc.px4
fi
