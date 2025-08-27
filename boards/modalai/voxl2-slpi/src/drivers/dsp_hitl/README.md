
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
