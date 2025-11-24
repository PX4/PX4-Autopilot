#!/usr/bin/env python

################################################################################################
# @File MissionCheck.py
# Automated mission loading, execution and monitoring
# for Continuous Integration
#
# @author Sander Smeets <sander@droneslab.com>
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################


################################################################################################
# Settings
################################################################################################

from __future__ import print_function
connection_string       = '127.0.0.1:14540'

import_mission_filename = 'VTOL_TAKEOFF.mission'
max_execution_time      = 200
alt_acceptance_radius   = 5

################################################################################################
# Init
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, json

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-f", "--filename", help="mission filename")
parser.add_argument("-t", "--timeout", help="execution timeout", type=float)
parser.add_argument("-a", "--altrad", help="altitude acceptance radius", type=float)
args = parser.parse_args()

if args.connect:
    connection_string = args.connect
if args.filename:
    import_mission_filename = args.filename
if args.timeout:
    max_execution_time = args.timeout
if args.altrad:
    alt_acceptance_radius = args.altrad



mission_failed = False
MAV_MODE_AUTO = 4

# start time counter
start_time = time.time()
elapsed_time = time.time() - start_time



# Connect to the Vehicle
print("Connecting")
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.system_status.state == "STANDBY" or vehicle.gps_0.fix_type < 3:
    if time.time() - start_time > 20:
        print("FAILED: SITL did not reach standby with GPS fix within 20 seconds")
        sys.exit(98)
    print("Waiting for vehicle to initialise... %s " % vehicle.system_status.state)
    time.sleep(1)

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" Armed?: %s" % vehicle.armed)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)


################################################################################################
# Functions
################################################################################################

def read_mission_json(f):
    d = json.load(f)
    current = True
    missionlist=[]
    for wp in d['items']:
        cmd = Command( 0, 0, 0, int(wp['frame']), int(wp['command']), current, int(wp['autoContinue']), float(wp['param1']), float(wp['param2']), float(wp['param3']), float(wp['param4']), float(wp['coordinate'][0]), float(wp['coordinate'][1]), float(wp['coordinate'][2]))
        missionlist.append(cmd)
        if current:
            current = False
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    with open(aFileName) as f:
        missionlist = read_mission_json(f)

    #Clear existing mission from vehicle
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Uploaded mission with %s items' % len(missionlist))
    vehicle.commands.upload()
    return missionlist



################################################################################################
# Listeners
################################################################################################

current_sequence = -1
current_sequence_changed = False
current_landed_state = -1
home_position_set = False

#Create a message listener for mission sequence number
@vehicle.on_message('MISSION_CURRENT')
def listener(self, name, mission_current):
    global current_sequence, current_sequence_changed
    if (current_sequence != mission_current.seq):
        current_sequence = mission_current.seq;
        current_sequence_changed = True
        print('current mission sequence: %s' % mission_current.seq)

#Create a message listener for mission sequence number
@vehicle.on_message('EXTENDED_SYS_STATE')
def listener(self, name, extended_sys_state):
    global current_landed_state
    if (current_landed_state != extended_sys_state.landed_state):
        current_landed_state = extended_sys_state.landed_state;

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True



################################################################################################
# Start mission test
################################################################################################


while not home_position_set:
    if time.time() - start_time > 30:
        print("FAILED: getting home position 30 seconds")
        sys.exit(98)
    print("Waiting for home position...")
    time.sleep(1)


#Upload mission from file
missionlist = upload_mission(import_mission_filename)
time.sleep(2)

# set mission mode
vehicle.mode = VehicleMode("MISSION")
time.sleep(1)


# Arm vehicle
vehicle.armed = True

while not vehicle.system_status.state == "ACTIVE":
    if time.time() - start_time > 30:
        print("FAILED: vehicle did not arm within 30 seconds")
        sys.exit(98)
    print("Waiting for vehicle to arm...")
    time.sleep(1)



# Wait for completion of mission items
while (current_sequence < len(missionlist)-1 and elapsed_time < max_execution_time):
    time.sleep(.2)
    if current_sequence > 0 and current_sequence_changed:

        if missionlist[current_sequence-1].z - alt_acceptance_radius > vehicle.location.global_relative_frame.alt or missionlist[current_sequence-1].z + alt_acceptance_radius < vehicle.location.global_relative_frame.alt:
            print("waypoint %s out of bounds altitude %s gps altitude: %s" % (current_sequence, missionlist[current_sequence-1].z, vehicle.location.global_relative_frame.alt))
            mission_failed = True
        current_sequence_changed = False
    elapsed_time = time.time() - start_time

if elapsed_time < max_execution_time:
    print("Mission items have been executed")

# wait for the vehicle to have landed
while (current_landed_state != 1 and elapsed_time < max_execution_time):
    time.sleep(1)
    elapsed_time = time.time() - start_time

if elapsed_time < max_execution_time:
    print("Vehicle has landed")

# Disarm vehicle
vehicle.armed = False

# count elapsed time
elapsed_time = time.time() - start_time

# Close vehicle object before exiting script
vehicle.close()
time.sleep(2)

# Validate time constraint
if elapsed_time <= max_execution_time and not mission_failed:
    print("Mission succesful time elapsed %s" % elapsed_time)
    sys.exit(0)

if elapsed_time > max_execution_time:
    print("Mission FAILED to execute within %s seconds" % max_execution_time)
    sys.exit(99)

if mission_failed:
    print("Mission FAILED out of bounds")
    sys.exit(100)

print("Mission FAILED something strange happened")
sys.exit(101)
