# Vehicle Command uORB message. Used for commanding a mission / action / etc.
# Follows the MAVLink COMMAND_INT / COMMAND_LONG definition

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start.

uint16 VEHICLE_CMD_CUSTOM_0 = 0 # Test command.
uint16 VEHICLE_CMD_CUSTOM_1 = 1 # Test command.
uint16 VEHICLE_CMD_CUSTOM_2 = 2 # Test command.
uint16 VEHICLE_CMD_NAV_WAYPOINT = 16 # Navigate to MISSION. |[s] (decimal) Hold time. (ignored by fixed wing, time to stay at MISSION for rotary wing)|[m] Acceptance radius (if the sphere with this radius is hit, the MISSION counts as reached)|0 to pass through the WP, if > 0 radius [m] to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.|Desired yaw angle at MISSION (rotary wing)|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_UNLIM = 17 # Loiter around this MISSION an unlimited amount of time. |Unused|Unused|[m] Radius around MISSION. If positive loiter clockwise, else counter-clockwise|Desired yaw angle.|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_TURNS = 18 # Loiter around this MISSION for X turns. |Turns|Unused|Radius around MISSION [m]. If positive loiter clockwise, else counter-clockwise|Desired yaw angle.|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_TIME = 19 # Loiter around this MISSION for time. |[s] Seconds (decimal)|Unused|Radius around MISSION [m]. If positive loiter clockwise, else counter-clockwise|Desired yaw angle.|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20 # Return to launch location. |Unused|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_NAV_LAND = 21 # Land at location. |Unused|Unused|Unused|Desired yaw angle.|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_TAKEOFF = 22 # Takeoff from ground / hand. |Unused (FW pitch from FW_TKO_PITCH_MIN)|Unused|Unused|[deg] [@range 0,360] Yaw angle in NED if yaw source available, ignored otherwise|Latitude (WGS-84)|Longitude (WGS-84)|[m] Altitude AMSL|
uint16 VEHICLE_CMD_NAV_PRECLAND = 23 # Attempt a precision landing.
uint16 VEHICLE_CMD_DO_ORBIT = 34 # Start orbiting on the circumference of a circle defined by the parameters. |[m] Radius|[m/s] Velocity|[@enum ORBIT_YAW_BEHAVIOUR] Yaw behaviour|Unused|Latitude/X|Longitude/Y|Altitude/Z|
uint16 VEHICLE_CMD_DO_FIGUREEIGHT = 35 # Start flying on the outline of a figure eight defined by the parameters. |[m] Major radius|[m] Minor radius|[m/s] Velocity|Orientation|Latitude/X|Longitude/Y|Altitude/Z|
uint16 VEHICLE_CMD_NAV_ROI = 80 # Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |[@enum VEHICLE_ROI] Region of interest mode.|MISSION index/ target ID.|ROI index (allows a vehicle to manage multiple ROI's)|Unused|x the location of the fixed ROI (see MAV_FRAME)|y|z|
uint16 VEHICLE_CMD_NAV_PATHPLANNING = 81 # Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning|0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid|Unused|[deg] [@range 0, 360] Yaw angle at goal, in compass degrees|Latitude/X of goal|Longitude/Y of goal|Altitude/Z of goal|
uint16 VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84 # Takeoff from ground / hand and transition to fixed wing. |Minimum pitch (if airspeed sensor present), desired pitch without sensor|Transition heading, 0: Default, 3: Use specified transition heading|Unused|Yaw angle (if magnetometer present), ignored without magnetometer|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_VTOL_LAND = 85 # Transition to MC and land at location. |Unused|Unused|Unused|Desired yaw angle.|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_NAV_GUIDED_LIMITS = 90 # Set limits for external control. |[s] Timeout  - maximum time that external controller will be allowed to control vehicle. 0 means no timeout|[m] Absolute altitude min AMSL - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit|[m] Absolute altitude max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit|[m] Horizontal move limit (AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit|Unused|Unused|Unused|
uint16 VEHICLE_CMD_NAV_GUIDED_MASTER = 91 # Set id of master controller. |System ID|Component ID|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_NAV_DELAY = 93 # Delay the next navigation command a number of seconds or until a specified time. |[s] Delay (decimal, -1 to enable time-of-day fields)|[h] hour (24h format, UTC, -1 to ignore)|minute (24h format, UTC, -1 to ignore)|second (24h format, UTC)|Unused|Unused|Unused|
uint16 VEHICLE_CMD_NAV_LAST = 95 # NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration.|Unused|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_CONDITION_DELAY = 112 # Delay mission state machine. |[s] Delay (decimal seconds)|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_CONDITION_CHANGE_ALT = 113 # Ascend/descend at rate. Delay mission state machine until desired altitude reached.|Descent / Ascend rate (m/s)|Unused|Unused|Unused|Unused|Unused|Finish Altitude|
uint16 VEHICLE_CMD_CONDITION_DISTANCE = 114 # Delay mission state machine until within desired distance of next NAV point. |Distance [m]|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_CONDITION_YAW = 115 # Reach a certain target angle. |[deg] [@range 0,360] Target angle. 0 is north|[deg/s] Speed during yaw change|[@range -1,1] Direction: negative: counter clockwise, positive: clockwise |[ 1,0] Relative offset or absolute angle|Unused|Unused|Unused|
uint16 VEHICLE_CMD_CONDITION_LAST = 159 # NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration. |Unused|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_CONDITION_GATE = 4501 # Wait until passing a threshold. |2D coord mode: 0: Orthogonal to planned route|Altitude mode: 0: Ignore altitude|Unused|Unused|Lat|Lon|Alt|
uint16 VEHICLE_CMD_DO_SET_MODE = 176 # Set system mode. |Mode, as defined by ENUM MAV_MODE|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_JUMP = 177 # Jump to the desired command in the mission list. Repeat this action only the specified number of times. |Sequence number|Repeat count|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_CHANGE_SPEED = 178 # Change speed and/or throttle set points. |[@enum SPEED_TYPE] Speed type (0=Airspeed, 1=Ground Speed)|Speed (m/s, -1 indicates no change)|[%] Throttle ( Percent, -1 indicates no change)|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_SET_HOME = 179 # Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)|Unused|Unused|Unused|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_DO_SET_PARAMETER = 180 # Set a system parameter. Caution! Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number|Parameter value|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_SET_RELAY = 181 # Set a relay to a condition. |Relay number|Setting (1=on, 0=off, others possible depending on system hardware)|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_REPEAT_RELAY = 182 # Cycle a relay on and off for a desired number of cycles with a desired period. |Relay number|Cycle count|[s] Cycle time (decimal seconds)|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_REPEAT_SERVO = 184 # Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number|[us] PWM rate (1000 to 2000 typical)|Cycle count|[s] Cycle time|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_FLIGHTTERMINATION = 185 # Terminate flight immediately. |Flight termination activated if > 0.5|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_CHANGE_ALTITUDE = 186 # Set the vehicle to Loiter mode and change the altitude to specified value. |Altitude|Frame of new altitude|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_SET_ACTUATOR = 187 # Sets actuators (e.g. servos) to a desired value. |Actuator 1|Actuator 2|Actuator 3|Actuator 4|Actuator 5|Actuator 6|Index|
uint16 VEHICLE_CMD_DO_LAND_START = 189 # Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Unused|Unused|Unused|Unused|Latitude|Longitude|Unused|
uint16 VEHICLE_CMD_DO_GO_AROUND = 191 # Mission command to safely abort an autonomous landing. |[m] Altitude|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_REPOSITION = 192 # Reposition to specific WGS84 GPS position. |[m/s] Ground speed|Bitmask|[m] Loiter radius for planes|[deg] Yaw|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
uint16 VEHICLE_CMD_DO_SET_ROI_LOCATION = 195 # Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Unused|Unused|Unused|Unused|Latitude|Longitude|Altitude|
uint16 VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196 # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Unused|Unused|Unused|Unused|Pitch offset from next waypoint|Roll offset from next waypoint|Yaw offset from next waypoint|
uint16 VEHICLE_CMD_DO_SET_ROI_NONE = 197 # Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Unused|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_CONTROL_VIDEO = 200 # Control onboard camera system. |Camera ID (-1 for all)|Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw|Transmission mode: 0: video stream, >0: single images every n seconds (decimal seconds)|Recording: 0: disabled, 1: enabled compressed, 2: enabled raw|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_SET_ROI = 201 # Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |[@enum VEHICLE_ROI] Region of interest mode.|MISSION index/ target ID.|ROI index (allows a vehicle to manage multiple ROI's)|Unused|x the location of the fixed ROI (see MAV_FRAME)|y|z|
uint16 VEHICLE_CMD_DO_DIGICAM_CONTROL=203
uint16 VEHICLE_CMD_DO_MOUNT_CONFIGURE=204 # Mission command to configure a camera or antenna mount. |[@enum MAV_MOUNT_MODE] Mount operation mode|Stabilize roll? (1 = yes, 0 = no)|Stabilize pitch? (1 = yes, 0 = no)|stabilize yaw? (1 = yes, 0 = no)|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_MOUNT_CONTROL=205 # Mission command to control a camera or antenna mount. |[deg] Pitch or lat, depending on mount mode.|[deg] Roll or lon depending on mount mode|[deg]/[m] Yaw or alt depending on mount mode|Unused|Unused|Unused|[@enum MAV_MOUNT_MODE]|
uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST=206 # Mission command to set TRIG_DIST for this flight. |[m] Camera trigger distance|[ms] Shutter integration time|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_FENCE_ENABLE=207 # Mission command to enable the geofence. |enable? (0=disable, 1=enable)|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_PARACHUTE=208 # Mission command to trigger a parachute. |action [@enum PARACHUTE_ACTION] (0=disable, 1=enable, 2=release, for some systems see [@enum PARACHUTE_ACTION], not in general message set.)|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_MOTOR_TEST=209 # Motor test command. |Instance (@range 1, )|throttle type|throttle|timeout [s]|Motor count|Test order|Unused|
uint16 VEHICLE_CMD_DO_INVERTED_FLIGHT=210 # Change to/from inverted flight. |inverted (0=normal, 1=inverted)|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_GRIPPER = 211 # Command to operate a gripper.
uint16 VEHICLE_CMD_DO_AUTOTUNE_ENABLE = 212 # Enable autotune module. |1 to enable|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL=214 # Mission command to set TRIG_INTERVAL for this flight. |[m] Camera trigger distance|Shutter integration time (ms)|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT=220 # Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)|q2 - quaternion param #2, x (0 in null-rotation)|q3 - quaternion param #3, y (0 in null-rotation)|q4 - quaternion param #4, z (0 in null-rotation)|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_GUIDED_MASTER=221 # Set id of master controller. |System ID|Component ID|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_GUIDED_LIMITS=222 # Set limits for external control. |[s] Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout|[m] Absolute altitude min(AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit|[m] Absolute altitude max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit|[m] Horizontal move limit (AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit|Unused|Unused|Unused|
uint16 VEHICLE_CMD_DO_LAST = 240 # NOP - This command is only used to mark the upper limit of the DO commands in the enumeration. |Unused|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241 # Trigger calibration. This command will be only accepted if in pre-flight mode. See MAVLink spec MAV_CMD_PREFLIGHT_CALIBRATION.
uint16 PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION = 3# Param value for VEHICLE_CMD_PREFLIGHT_CALIBRATION to start temperature calibration.
uint16 VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242 # Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow|X axis offset (or generic dimension 1), in the sensor's raw units|Y axis offset (or generic dimension 2), in the sensor's raw units|Z axis offset (or generic dimension 3), in the sensor's raw units|Generic dimension 4, in the sensor's raw units|Generic dimension 5, in the sensor's raw units|Generic dimension 6, in the sensor's raw units|
uint16 VEHICLE_CMD_PREFLIGHT_UAVCAN = 243 # UAVCAN configuration. If param 1 == 1 actuator mapping and direction assignment should be started.
uint16 VEHICLE_CMD_PREFLIGHT_STORAGE = 245 # Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM|Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246 # Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.|0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_OBLIQUE_SURVEY=260 # Mission command to set a Camera Auto Mount Pivoting Oblique Survey for this flight. |[m] Camera trigger distance|[ms] Shutter integration time|Camera minimum trigger interval|Number of positions|Roll|Pitch|Unused|
uint16 VEHICLE_CMD_DO_SET_STANDARD_MODE=262 # Enable the specified standard MAVLink mode. |MAV_STANDARD_MODE|
uint16 VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION = 283 # Command to ask information about a low level gimbal.

uint16 VEHICLE_CMD_MISSION_START = 300 # Start running a mission. |first_item: the first mission item to run|last_item: the last mission item to run (after this item is run, the mission ends)|
uint16 VEHICLE_CMD_ACTUATOR_TEST = 310 # Actuator testing command. |[@range -1,1] value|[s] timeout|Unused|Unused|output function|
uint16 VEHICLE_CMD_CONFIGURE_ACTUATOR = 311 # Actuator configuration command. |configuration|Unused|Unused|Unused|output function|
uint16 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400 # Arms / Disarms a component. |1 to arm, 0 to disarm.
uint16 VEHICLE_CMD_RUN_PREARM_CHECKS = 401 # Instructs a target system to run pre-arm checks.
uint16 VEHICLE_CMD_INJECT_FAILURE = 420 # Inject artificial failure for testing purposes.
uint16 VEHICLE_CMD_START_RX_PAIR = 500 # Starts receiver pairing. |0:Spektrum|0:Spektrum DSM2, 1:Spektrum DSMX|
uint16 VEHICLE_CMD_REQUEST_MESSAGE = 512 # Request to send a single instance of the specified message.
uint16 VEHICLE_CMD_REQUEST_CAMERA_INFORMATION = 521 # Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities|Reserved (all remaining params)|Reserved (default:0)|Reserved (default:0)|Reserved (default:0)|Reserved (default:0)|Reserved (default:0)|
uint16 VEHICLE_CMD_SET_CAMERA_MODE = 530 # Set camera capture mode (photo, video, etc.).
uint16 VEHICLE_CMD_SET_CAMERA_ZOOM = 531 # Set camera zoom.
uint16 VEHICLE_CMD_SET_CAMERA_FOCUS = 532
uint16 VEHICLE_CMD_EXTERNAL_ATTITUDE_ESTIMATE = 620 # Set an external estimate of vehicle attitude in degrees.
uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000 # Setpoint to be sent to a gimbal manager to set a gimbal pitch and yaw.
uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001 # Gimbal configuration to set which sysid/compid is in primary and secondary control.
uint16 VEHICLE_CMD_IMAGE_START_CAPTURE = 2000 # Start image capture sequence.
uint16 VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003 # Enable or disable on-board camera triggering system.
uint16 VEHICLE_CMD_VIDEO_START_CAPTURE = 2500 # Start a video capture.
uint16 VEHICLE_CMD_VIDEO_STOP_CAPTURE = 2501 # Stop the current video capture.
uint16 VEHICLE_CMD_LOGGING_START = 2510 # Start streaming ULog data.
uint16 VEHICLE_CMD_LOGGING_STOP = 2511 # Stop streaming ULog data.
uint16 VEHICLE_CMD_CONTROL_HIGH_LATENCY = 2600 # Control starting/stopping transmitting data over the high latency link.
uint16 VEHICLE_CMD_DO_VTOL_TRANSITION = 3000 # Command VTOL transition.
uint16 VEHICLE_CMD_DO_SET_SAFETY_SWITCH_STATE = 5300 # Command safety on/off. |1 to activate safety, 0 to deactivate safety and allow control surface movements|Unused|Unused|Unused|Unused|Unused|Unused|
uint16 VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST = 3001 # Request arm authorization.
uint16 VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001 # Prepare a payload deployment in the flight plan.
uint16 VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002 # Control a pre-programmed payload deployment.
uint16 VEHICLE_CMD_FIXED_MAG_CAL_YAW = 42006 # Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
uint16 VEHICLE_CMD_DO_WINCH = 42600 # Command to operate winch.

uint16 VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE = 43003 # External reset of estimator global position when dead reckoning.
uint16 VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE = 43004

# PX4 vehicle commands (beyond 16 bit MAVLink commands).
uint32 VEHICLE_CMD_PX4_INTERNAL_START = 65537 # Start of PX4 internal only vehicle commands (> UINT16_MAX).
uint32 VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000 # Sets the GPS coordinates of the vehicle local origin (0,0,0) position. |Unused|Unused|Unused|Unused|Latitude (WGS-84)|Longitude (WGS-84)|[m] Altitude (AMSL from GNSS, positive above ground)|
uint32 VEHICLE_CMD_SET_NAV_STATE = 100001 # Change mode by specifying nav_state directly. |nav_state|Unused|Unused|Unused|Unused|Unused|Unused|

uint8 VEHICLE_MOUNT_MODE_RETRACT = 0 # Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization.
uint8 VEHICLE_MOUNT_MODE_NEUTRAL = 1 # Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
uint8 VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2 # Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization.
uint8 VEHICLE_MOUNT_MODE_RC_TARGETING = 3 # Load neutral position and start RC Roll,Pitch,Yaw control with stabilization.
uint8 VEHICLE_MOUNT_MODE_GPS_POINT = 4 # Load neutral position and start to point to Lat,Lon,Alt.
uint8 VEHICLE_MOUNT_MODE_ENUM_END = 5 #

uint8 VEHICLE_ROI_NONE = 0 # No region of interest.
uint8 VEHICLE_ROI_WPNEXT = 1 # Point toward next MISSION.
uint8 VEHICLE_ROI_WPINDEX = 2 # Point toward given MISSION.
uint8 VEHICLE_ROI_LOCATION = 3 # Point toward fixed location.
uint8 VEHICLE_ROI_TARGET = 4 # Point toward target.
uint8 VEHICLE_ROI_ENUM_END = 5

uint8 PARACHUTE_ACTION_DISABLE = 0
uint8 PARACHUTE_ACTION_ENABLE = 1
uint8 PARACHUTE_ACTION_RELEASE = 2

uint8 FAILURE_UNIT_SENSOR_GYRO = 0
uint8 FAILURE_UNIT_SENSOR_ACCEL = 1
uint8 FAILURE_UNIT_SENSOR_MAG = 2
uint8 FAILURE_UNIT_SENSOR_BARO = 3
uint8 FAILURE_UNIT_SENSOR_GPS = 4
uint8 FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
uint8 FAILURE_UNIT_SENSOR_VIO = 6
uint8 FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
uint8 FAILURE_UNIT_SENSOR_AIRSPEED = 8
uint8 FAILURE_UNIT_SYSTEM_BATTERY = 100
uint8 FAILURE_UNIT_SYSTEM_MOTOR = 101
uint8 FAILURE_UNIT_SYSTEM_SERVO = 102
uint8 FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
uint8 FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
uint8 FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105

uint8 FAILURE_TYPE_OK = 0
uint8 FAILURE_TYPE_OFF = 1
uint8 FAILURE_TYPE_STUCK = 2
uint8 FAILURE_TYPE_GARBAGE = 3
uint8 FAILURE_TYPE_WRONG = 4
uint8 FAILURE_TYPE_SLOW = 5
uint8 FAILURE_TYPE_DELAYED = 6
uint8 FAILURE_TYPE_INTERMITTENT = 7

# Used as param1 in DO_CHANGE_SPEED command.
uint8 SPEED_TYPE_AIRSPEED = 0
uint8 SPEED_TYPE_GROUNDSPEED = 1
uint8 SPEED_TYPE_CLIMB_SPEED = 2
uint8 SPEED_TYPE_DESCEND_SPEED = 3

# Used as param3 in CMD_DO_ORBIT.
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4
uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5

# Used as param1 in ARM_DISARM command.
int8 ARMING_ACTION_DISARM = 0
int8 ARMING_ACTION_ARM = 1

# param2 in VEHICLE_CMD_DO_GRIPPER.
uint8 GRIPPER_ACTION_RELEASE = 0
uint8 GRIPPER_ACTION_GRAB = 1

# Used as param1 in DO_SET_SAFETY_SWITCH_STATE command.
uint8 SAFETY_OFF = 0
uint8 SAFETY_ON = 1

uint8 ORB_QUEUE_LENGTH = 8

float32 param1 # Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param2 # Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param3 # Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param4 # Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param5 # Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param6 # Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param7 # Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
uint32 command # Command ID.
uint8 target_system # System which should execute the command.
uint8 target_component # Component which should execute the command, 0 for all components.
uint8 source_system # System sending the command.
uint16 source_component # Component / mode executor sending the command.
uint8 confirmation # 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command).
bool from_external

uint16 COMPONENT_MODE_EXECUTOR_START = 1000

# TOPICS vehicle_command gimbal_v1_command vehicle_command_mode_executor
