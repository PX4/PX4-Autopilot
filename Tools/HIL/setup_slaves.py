#! /usr/bin/python

import json
import subprocess
import get_device_info_from_mavlink as dm
import get_device_info_from_serial as ds
from pprint import pprint

def find_devices():
    available_devices = subprocess.check_output(['bash','-c', './finddevice.sh'])
    return json.loads(available_devices)

def get_device(device_list, uid):
    found_ftdi = False
    found_usb = False
    device_usb = None
    device_ftdi = None

    for device in device_list:
        if (device['UID'] == uid) & (device['connector'] == 'FTDI'):
            found_ftdi = True
            device_ftdi = device
        elif (device['UID'] == uid) & (device['connector'] == 'USB'):
            found_usb = True
            device_usb = device

    return (found_ftdi & found_usb), device_usb, device_ftdi

def init_slave(device_usb, device_ftdi, config):
    print("Initializing slaves...")
    docker_build_output = subprocess.check_output(['bash','-c', 'docker build -t blueocean_slave .'])

    if 'Successfully built ' in docker_build_output:
        print('Successfully built image')
    else:
        print('Error occured while building docker image:\n')
        print(docker_build_output)

    docker_run_cmd = 'docker run -itd --privileged \
                      -v /home/dario/jenkins_slave_home/:/home/hil/:rw \
                      -e "USB_DEVICE="' + str(device_usb['device']) + ' \
                      -e "USB_HUB_BUS="' + str(device_usb['bus_hub']) + ' \
                      -e "USB_HUB_DEVICE="' + str(device_usb['device_hub']) + ' \
                      -e "USB_HUB_PORT="' + str(device_usb['port']) + ' \
                      -e "FTDI_DEVICE="' + str(device_ftdi['device']) + ' \
                      -e "FTDI_HUB_BUS="' + str(device_ftdi['bus_hub']) + ' \
                      -e "FTDI_HUB_DEVICE="' + str(device_ftdi['device_hub']) + ' \
                      -e "FTDI_HUB_PORT="' + str(device_ftdi['port']) + ' \
                      -e "HIL_HW="' + device_usb['HW'] + ' \
                      -e "JENKINS_SLAVE_ID="' + str(config['JENKINS_SLAVE_ID']) + ' \
                      -e "JENKINS_SLAVE_SECRET="' + config['JENKINS_SLAVE_SECRET'] + ' \
                      -e "JENKINS_MASTER_URL="' + config['JENKINS_MASTER_URL'] + ' \
                      --name HIL_SLAVE_' + device_usb['HW'] + ' \
                      blueocean_slave'

                    # --device ' + device_usb['device'] + ' \
                    # --device ' + device_ftdi['device'] + ' \
    # print(docker_run_cmd)
    docker_run_output = subprocess.check_output(['bash','-c', docker_run_cmd])
    print(docker_run_output)

def stop_and_remove_all_slaves():
    container_name='HIL_SLAVE_*'

    slaves_list = subprocess.check_output(['bash','-c', 'docker ps -a -q -f name="' + container_name + '"'])
    if len(slaves_list) > 0:
        print("Stop and remove slaves:")
        for slave in slaves_list.splitlines():
            if len(slave) > 0:
                print("\t" + slave)

        subprocess.check_output(['bash','-c', 'docker stop $(docker ps -a -q -f name="' + container_name + '")'])
        subprocess.check_output(['bash','-c', 'docker rm $(docker ps -a -q -f name="' + container_name + '")'])

def main():
    stop_and_remove_all_slaves()

    device_list = find_devices()
    for device in device_list:
        if device['connector'] == 'FTDI':
            device_info = ds.get_device_info_from_serial(device['device'], 57600)
        elif device['connector'] == 'USB':
            device_info = dm.get_device_info_from_mavlink(device['device'], 115200)
        else:
            continue

        device['UID'] = device_info['UID']
        device['HW'] = device_info['HW']

    with open('device_config.json') as device_config_file:    
        configured_devices = json.load(device_config_file)
        found=False
        for device in configured_devices:
            device_found, device_usb, device_ftdi = get_device(device_list, device['UID'])
            if device_found:
                found=True
                print("Configured device found:\n\tHW: " + device['HW'] + "\n\tUID: " + device['UID'])
                init_slave(device_usb, device_ftdi, device)

        if not found:
            print("No configured device found! Make sure it is connected over USB and FTDI.")

if __name__ == '__main__':
    main()
