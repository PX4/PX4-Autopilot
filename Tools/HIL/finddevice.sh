#!/bin/bash

filename="devices.json"

echo -e "[" > devices.json


for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
(
    syspath="${sysdevpath%/dev}"
    devname="$(udevadm info -q name -p $syspath)"

    [[ "$devname" == "bus/"* ]] && continue
    eval "$(udevadm info -q property --export -p $syspath)"
    [[ -z "$ID_SERIAL" ]] && continue
    [[ "$ID_SERIAL" == *"_PX4_"* ]] || [[ "$ID_SERIAL" == *"FTDI"* ]] || continue

    port_type="USB"
    levels_up_to_hub="../../../"
    [[ "$ID_SERIAL" == *"FTDI"* ]] && levels_up_to_hub="$levels_up_to_hub../" && port_type="FTDI"

    syspath="${sysdevpath%/dev}"
    devname="$(udevadm info -q name -p $syspath)"
    hub_dev="$(cat $syspath/${levels_up_to_hub}../devnum)"
    hub_bus="$(cat $syspath/${levels_up_to_hub}../busnum)"
    device_devnum="$(cat $syspath/${levels_up_to_hub}devnum)"
    device_port="$(lsusb -t | grep "Dev $device_devnum" | head -n 1 | sed 's/.*Port \([0-9]*\).*/\1/')";

    echo -e "\t{\n\t\t\"bus_hub\": $hub_bus,\n\t\t\"device_hub\": $hub_dev,\n\t\t\"port\": $device_port,\n\t\t\"device\": \"/dev/$devname\",\n\t\t\"connector\": \"$port_type\"\n\t}," >> devices.json
)
done

echo -e "]" >> devices.json
sed -i -e 'x;${s/,$//;p;x;};1d' devices.json
cat devices.json
