################################################################################################
# @File MissionCheck.py
# Automated mission loading, execution and monitoring
# for Continuous Integration
#
# @author Sander Smeets <sander@droneslab.com>
#
################################################################################################


################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540'

import_mission_filename = 'VTOLmission.txt'
max_execution_time      = 250


################################################################################################
# Init
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command
from pymavlink import mavutil
import time, sys

MAV_MODE_AUTO = 4

# start time counter
start_time = time.time()
elapsed_time = time.time() - start_time

# Connect to the Vehicle
print "Connecting"
vehicle = connect(connection_string, wait_ready=True)
vehicle.armed = False

while not vehicle.system_status.state == "STANDBY":
    print " Waiting for vehicle to initialise... %s " % vehicle.system_status.state
    time.sleep(1)


################################################################################################
# Functions
################################################################################################


def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print "\nReading mission from file: %s" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName)

    print "\nUpload mission from a file: %s" % import_mission_filename
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission with %s items' % len(missionlist)
    vehicle.commands.upload()
    return missionlist


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print " Download mission from vehicle"
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print "\nSave mission from Vehicle to file: %s" % export_mission_filename
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print " Write mission to file"
        file_.write(output)


def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print "\nMission file: %s" % aFileName
    with open(aFileName) as f:
        for line in f:
            print ' %s' % line.strip()

################################################################################################
# Listeners
################################################################################################

current_sequence = -1
current_landed_state = -1

#Create a message listener for mission sequence number
@vehicle.on_message('MISSION_CURRENT')
def listener(self, name, mission_current):
    global current_sequence
    if (current_sequence <> mission_current.seq):
        current_sequence = mission_current.seq;
        print 'current mission sequence: %s' % mission_current.seq

#Create a message listener for mission sequence number
@vehicle.on_message('EXTENDED_SYS_STATE')
def listener(self, name, extended_sys_state):
    global current_landed_state
    if (current_landed_state <> extended_sys_state.landed_state):
        current_landed_state = extended_sys_state.landed_state;
        print 'Landed state: %s' % extended_sys_state.landed_state


################################################################################################
# Start mission test
################################################################################################

#Upload mission from file
missionlist = upload_mission(import_mission_filename)
time.sleep(2)

# set mission mode the hard way
vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           MAV_MODE_AUTO,
                                           0, 0, 0, 0, 0, 0)
time.sleep(1)

# Arm vehicle
vehicle.armed = True

# Wait for completion of mission items
while (current_sequence < len(missionlist)-1 and elapsed_time < max_execution_time):
    time.sleep(1)
    elapsed_time = time.time() - start_time

print "Mission items have been executed"

# wait for the vehicle to have landed
while (current_landed_state != 1 and elapsed_time < max_execution_time):
    time.sleep(1)
    elapsed_time = time.time() - start_time

print "Vehicle has landed"

# Disarm vehicle
vehicle.armed = False

# count elapsed time
elapsed_time = time.time() - start_time

# Close vehicle object before exiting script
vehicle.close()

# Validate time constraint
if elapsed_time < max_execution_time:
    print "Mission succesful time elapsed %s" % elapsed_time
    sys.exit(0)

print "Mission FAILED to execute within %s seconds time elapsed %s" % max_execution_time % elapsed_time
sys.exit(99)
