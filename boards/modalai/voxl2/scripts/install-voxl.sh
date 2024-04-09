#!/bin/bash

# Push slpi image to voxl2
adb push build/modalai_voxl2-slpi_default/platforms/qurt/libpx4.so /usr/lib/rfsa/adsp

# Push apps processor image to voxl2
adb push build/modalai_voxl2_default/bin/px4 /usr/bin

# Push scripts to voxl2
adb push build/modalai_voxl2_default/bin/px4-alias.sh /usr/bin
adb push boards/modalai/voxl2/target/voxl-px4 /usr/bin
adb push boards/modalai/voxl2/target/voxl-px4-start /usr/bin
adb push boards/modalai/voxl2/target/voxl-px4-hitl /usr/bin
adb push boards/modalai/voxl2/target/voxl-px4-hitl-start /usr/bin
adb shell chmod a+x /usr/bin/px4-alias.sh
adb shell chmod a+x /usr/bin/voxl-px4
adb shell chmod a+x /usr/bin/voxl-px4-start
adb shell chmod a+x /usr/bin/voxl-px4-hitl
adb shell chmod a+x /usr/bin/voxl-px4-hitl-start

# Push configuration file
adb shell mkdir -p /etc/modalai
adb push boards/modalai/voxl2/target/voxl-px4-set-default-parameters.config /etc/modalai
adb push boards/modalai/voxl2/target/voxl-px4-fake-imu-calibration.config /etc/modalai
adb push boards/modalai/voxl2/target/voxl-px4-hitl-set-default-parameters.config /etc/modalai

# Make sure to setup all of the needed px4 aliases.
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-accelsim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-attitude_estimator_q"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-barosim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-batt_smbus"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-bottle_drop"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-camera_feedback"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-camera_trigger"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-cdev_test"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-cm8jl65"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-commander"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-commander_tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-control_allocator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-controllib_test"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-dataman"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-dsp_hitl"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ekf2"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-esc_calib"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ets_airspeed"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ex_fixedwing_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-fw_att_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-fw_pos_control_l1"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-gnd_att_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-gnd_pos_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-gps"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-gpssim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-gyrosim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-hello"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-hrt_test"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-land_detector"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-landing_target_estimator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-led_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-leddar_one"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-linux_sbus"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-listener"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ll40ls"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-load_mon"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-local_position_estimator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-logger"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-manual_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mavlink"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mavlink_bridge"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mavlink_tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mb12xx"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mc_att_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mc_pos_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-measairspeedsim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-microdds_client"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-mixer"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-voxl2_io_bridge"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-motor_ramp"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-modalai_gps_timer"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ms4525_airspeed"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ms5525_airspeed"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-msp_osd"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-muorb"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-muorb_test"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-navigator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-param"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-perf"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-pga460"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-position_estimator_inav"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-pwm"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-pwm_out_sim"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-px4io"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-px4_mavlink_debug"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-px4_simple_app"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-qshell"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-rc_tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-rc_update"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-reboot"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-rgbled"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-rover_steering_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sd_bench"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sdp3x_airspeed"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-segway"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-send_event"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sensors"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sf0x"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sf0x_tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sf1xx"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-shutdown"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-sih"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-simulator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-spektrum_rc"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-srf02"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-teraranger"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-tfmini"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-top"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-tune_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ulanding_radar"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-uorb"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-uorb_tests"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-ver"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-vl53lxx"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-vmount"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-vtol_att_control"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-wind_estimator"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-rc_controller"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-uart_esc_driver"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-flight_mode_manager"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-imu_server"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-apps_sbus"
adb shell "cd /usr/bin; /bin/ln -f -s px4 px4-voxl_save_cal_params"

# Make sure any required directories exist
adb shell "/bin/mkdir -p /data/px4/param"
adb shell "/bin/mkdir -p /data/px4/etc/extras"

# Push the json files for the component metadata
adb push build/modalai_voxl2_default/actuators.json.xz /data/px4/etc/extras
adb push build/modalai_voxl2_default/component_general.json.xz /data/px4/etc/extras
adb push build/modalai_voxl2_default/parameters.json.xz /data/px4/etc/extras
adb push build/modalai_voxl2_default/events/all_events.json.xz /data/px4/etc/extras

adb shell sync
