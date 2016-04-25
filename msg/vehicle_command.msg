
uint32 VEHICLE_CMD_CUSTOM_0 = 0				# test command
uint32 VEHICLE_CMD_CUSTOM_1 = 1				# test command
uint32 VEHICLE_CMD_CUSTOM_2 = 2				# test command
uint32 VEHICLE_CMD_NAV_WAYPOINT = 16			# Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_LOITER_UNLIM = 17		# Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_LOITER_TURNS = 18		# Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_LOITER_TIME = 19			# Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20		# Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_NAV_LAND = 21			# Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_TAKEOFF = 22			# Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_NAV_ROI = 80				# Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z| 
uint32 VEHICLE_CMD_NAV_PATHPLANNING = 81		# Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal| 
uint32 VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84		# Takeoff from ground / hand and transition to fixed wing |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
uint32 VEHICLE_CMD_NAV_VTOL_LAND = 85			# Transition to MC and land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|
uint32 VEHICLE_CMD_NAV_GUIDED_LIMITS = 90		# set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_NAV_GUIDED_MASTER = 91		# set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_NAV_GUIDED_ENABLE = 92		# hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_NAV_LAST = 95			# NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_CONDITION_DELAY = 112		# Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_CONDITION_CHANGE_ALT = 113		# Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude| 
uint32 VEHICLE_CMD_CONDITION_DISTANCE = 114		# Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_CONDITION_YAW = 115			# Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_CONDITION_LAST = 159			# NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_SET_MODE = 176			# Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_JUMP = 177			# Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_CHANGE_SPEED = 178		# Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_SET_HOME = 179			# Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude| 
uint32 VEHICLE_CMD_DO_SET_PARAMETER = 180		# Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_SET_RELAY = 181			# Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_REPEAT_RELAY = 182		# Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_SET_SERVO = 183			# Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_REPEAT_SERVO = 184		# Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_FLIGHTTERMINATION=185		# Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_REPOSITION = 192
uint32 VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
uint32 VEHICLE_CMD_DO_CONTROL_VIDEO = 200		# Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_DIGICAM_CONTROL=203
uint32 VEHICLE_CMD_DO_MOUNT_CONFIGURE=204		# Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_MOUNT_CONTROL=205			# Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value| 
uint32 VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST=206		# Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_FENCE_ENABLE=207			# Mission command to enable the geofence |enable? (0=disable, 1=enable)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_PARACHUTE=208			# Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_INVERTED_FLIGHT=210		# Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT=220		# Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_GUIDED_MASTER=221			# set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_GUIDED_LIMITS=222			# set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_DO_LAST = 240			# NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241		# Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Empty| Empty| 
uint32 VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242	# Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units| 
uint32 VEHICLE_CMD_PREFLIGHT_STORAGE = 245		# Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246	# Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty| 
uint32 VEHICLE_CMD_OVERRIDE_GOTO = 252			# Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position| 
uint32 VEHICLE_CMD_MISSION_START = 300			# start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)| 
uint32 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400		# Arms / Disarms a component |1 to arm, 0 to disarm| 
uint32 VEHICLE_CMD_START_RX_PAIR = 500			# Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|
uint32 VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003            # Enable or disable on-board camera triggering system
uint32 VEHICLE_CMD_DO_VTOL_TRANSITION = 3000    # Command VTOL transition
uint32 VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001	# Prepare a payload deployment in the flight plan
uint32 VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002	# Control a pre-programmed payload deployment
uint32 VEHICLE_CMD_PREFLIGHT_UAVCAN = 243		# UAVCAN configuration. If param 1 == 1 actuator mapping and direction assignment should be started 

uint32 VEHICLE_CMD_RESULT_ACCEPTED = 0			# Command ACCEPTED and EXECUTED |
uint32 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1	# Command TEMPORARY REJECTED/DENIED |
uint32 VEHICLE_CMD_RESULT_DENIED = 2			# Command PERMANENTLY DENIED |
uint32 VEHICLE_CMD_RESULT_UNSUPPORTED = 3		# Command UNKNOWN/UNSUPPORTED |
uint32 VEHICLE_CMD_RESULT_FAILED = 4			# Command executed, but failed |
uint32 VEHICLE_CMD_RESULT_ENUM_END = 5			#

uint32 VEHICLE_MOUNT_MODE_RETRACT = 0				# Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization |
uint32 VEHICLE_MOUNT_MODE_NEUTRAL = 1				# Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. |
uint32 VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2			# Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization |
uint32 VEHICLE_MOUNT_MODE_RC_TARGETING = 3			# Load neutral position and start RC Roll,Pitch,Yaw control with stabilization |
uint32 VEHICLE_MOUNT_MODE_GPS_POINT = 4				# Load neutral position and start to point to Lat,Lon,Alt |
uint32 VEHICLE_MOUNT_MODE_ENUM_END = 5				#

float32 param1			# Parameter 1, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float32 param2			# Parameter 2, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float32 param3			# Parameter 3, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float32 param4			# Parameter 4, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float64 param5			# Parameter 5, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float64 param6			# Parameter 6, as defined by MAVLink uint32 VEHICLE_CMD enum.  
float32 param7			# Parameter 7, as defined by MAVLink uint32 VEHICLE_CMD enum.  
uint32 command			# Command ID, as defined MAVLink by uint32 VEHICLE_CMD enum.  
uint32 target_system		# System which should execute the command 
uint32 target_component		# Component which should execute the command, 0 for all components 
uint32 source_system		# System sending the command 
uint32 source_component		# Component sending the command 
uint8 confirmation		# 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
