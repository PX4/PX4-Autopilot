
# Using HITL with VFC

## voxl-vision-hub

- Run it from command line, not as a service
- Start it after px4
- Run it with -u option to see debug messages
- Set en_hitl to true
- Set offboard_mode to vfc

## voxl-px4-hitl

- Add -e option to dsp_hitl start command to send odometry to voxl-vision-hub
- Set MAV_RC_BRIDGE to 1 for joystick to voxl-vision-hub routing

## Using joystick

- Set MAV_RC_BRIDGE to 1
- Set COM_RC_IN_MODE to 1
- VOXL_ESC_T_ON - Set to 10 to use right bumper button
- MAV_RC_FM1 - Set to 0 to use button 0. Set button 0 to altitude flight mode on joystick setup in QGC
- MAV_RC_FM2 - Set to 1 to use button 1. Set button 1 to offboard flight mode on joystick setup in QGC
- MAV_RC_FM3 - Set to 2 to use button 2. Set button 2 to rattitude flight mode on joystick setup in QGC
- Put toggle arm on button 9 to use left bumper button on joystick setup in QGC

- Set backtrack_rc_chan to 6 in vfc.conf file. Chan6_raw is set to contents of aux4 which is turtle mode (backtrack) button

