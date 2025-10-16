
# HITL scripts

- voxl-px4-hitl: Very simple, perhaps could be folded into voxl-px4
- voxl-px4-hitl-start: Should be able to fold this in to voxl-px4-start
- voxl-px4-hitl-set-default-parameters.config: Could be moved to standard method

- voxl-px4 -s is the new method to start

# HITL parameters

- HITL uses it's own parameter file so that the actual parameter file doesn't
get modified by HITL settings. It is located at /data/px4/param/hitl_parameters.

## Non-product specific configuration
- Stop voxl-px4
- rm /data/px4/param/hitl_parameters*
- voxl-px4 -s
- voxl-configure-px4-params -p HITL --skip-cal --quiet --non-interactive
- restart voxl-px4

## Product specific configuration
- Stop voxl-px4
- rm /data/px4/param/hitl_parameters*
- voxl-px4 -s
- voxl-configure-px4-params -p <product type> --skip-cal --quiet --non-interactive
- voxl-configure-px4-params -f /usr/share/modalai/px4_params/v1.14/platforms/HITL_Iris.params --quiet --non-interactive
- restart voxl-px4

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

## VFC PID and hover thrust

- "vfc_thrust_hover":   0.7
- "vfc_kp_z_vio": 0.7
- "vfc_ki_z_vio": 0.6
- "vfc_kd_z_vio": 0.35

- voxl-configure-vision-hub configure_hitl

# Using joystick

- Set MAV_RC_BRIDGE to 1
- Set COM_RC_IN_MODE to 1
- VOXL_ESC_T_ON - Set to 10 to use right bumper button
- MAV_RC_FM1 - Set to 0 to use button 0. Set button 0 to altitude flight mode on joystick setup in QGC
- MAV_RC_FM2 - Set to 1 to use button 1. Set button 1 to offboard flight mode on joystick setup in QGC
- MAV_RC_FM3 - Set to 2 to use button 2. Set button 2 to rattitude flight mode on joystick setup in QGC
- Put toggle arm on button 9 to use left bumper button on joystick setup in QGC

- voxl-configure-px4-params -f /usr/share/modalai/px4_params/v1.14/other_helpers/hitl_joystick_vfc.params --quiet --non-interactive

- Set backtrack_rc_chan to 6 in vfc.conf file. Chan6_raw is set to contents of aux4 which is turtle mode (backtrack) button

- voxl-configure-vision-hub configure_hitl_joystick


