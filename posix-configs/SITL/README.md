Running PX4 in SITL Mode
==========================

Introduction
--------------------------
This assumes that you have PX4 (posix configuration) and jMAVSim installed and compiled.  These can be on the same physical machine or on different machines as long as there is a network between the two.  The system has many possible configurations - this README will focus just on one to keep things simple.  Namely:

- The controller inputs are coming from a PIXHAWK that has an active connection to an RC controller and is generating the [Mavlink](https://pixhawk.ethz.ch/mavlink/) message RC_CHANNELS(#65) on its serial (USB) port.  This Pixhawk's USB is connected to a USB port on the machine running PX4.
- PX4 and jMAVSim communicate over UDP
- qGroundControl is connecting to PX4 over UDP

A diagram of the setup described is shown here.  Note that UDP port numbers are only displayed on the socket server and are left blank on the socket client. 

![SITL Diagram](SITL_Diagram.png "SITL Diagram")  

Steps
---------------------------

1. Connect the RC Controller (PIXHAWK) to the PX4 machine using USB.  Verify the `/dev/ttyACM0` device appears.  Make sure that the persmissions of this device allow the PX4 app to open the device for read/write (`sudo chmod 777 /dev/ttyACM0`).

1. Run the quadrotor simulation:
```
> make sitlrun
```

Detailed Background on System startup
---------------------------

NOTE: This is only necessary if you are not using the instructions above.

1. Connect the RC Controller (PIXHAWK) to the PX4 machine using USB.  Verify the `/dev/ttyACM0` device appears.  Make sure that the persmissions of this device allow the PX4 app to open the device for read/write (`sudo chmod 777 /dev/ttyACM0`).

1. Create the following diretories in "`./Firmware/Build/posix_sitl.build/`":
```
> cd ./Firmware/Build/posix_sitl.build
> mkdir -p rootfs/fs/microsd
> mkdir -p rootfs/eeprom
```

1. Launch PX4 from the SITL build directory "`./Firmware/Build/posix_sitl.build/`" using the command below.  The optional `<startup_file>` can be used to cause a set of commands to be entered into the PX4 shell at startup.  
```
> ./mainapp <startup_file> 
```

There is a sample startup script at `posix_configs/SITL/init/rcS`. 
```
> ./mainapp ../../posix-configs/SITL/init/rcS
```

Without the `<startup_file>`, the commands can be entered at the shell prompt one at a time.  An example startup file is given below.  This example shows that the "mavlink" module has selected port 14556 for its socket server (as shown in the SITL diagram) and will listen for connections on this port.

1. qGroundControl (QGC) can be connected to this instance of PX4 any time after it is started.  The instructions for configuring (QGC) are not given here in detail, but you just need to configure a UDP connection with ip_address:`<address of PX4 machine>` and port:14556.  (QGC also has a "Listening Port" setting that must be set to another unused port).

1. jMAVSim can now be launched to complete the setup.  At the root directory of the jMAVSim code, type the following command, replacing the IP address of the machine that is running PX4.  The port 14550 is the default port of the simulator module in PX4.
```
> java -cp lib/*:out/production/jmavsim.jar me.drton.jmavsim.Simulator -udp <IP_of_PX4_machine>:14550
```

At this point, you should see the jMAVSim world visualization, and when you increase the throttle on your RC controller, the vehicle will take off.  Note that the RC params in the example startup file may not be correct for your controller.

Example "startup" file
---------------------

```
uorb start
mavlink start -u 14556
simulator start -s
param set CAL_GYRO0_ID 2293760
param set CAL_ACC0_ID 1310720
param set CAL_ACC1_ID 1376256
param set CAL_MAG0_ID 196608
rgbled start
tone_alarm start
gyrosim start
accelsim start
barosim start
adcsim start
commander start
sensors start
ekf_att_pos_estimator start
mc_pos_control start
mc_att_control start
hil mode_pwm
param set MAV_TYPE 2
param set RC1_MAX 2015
param set RC1_MIN 996
param set RC1_TRIM 1502
param set RC1_REV -1
param set RC2_MAX 2016 
param set RC2_MIN 995
param set RC2_TRIM 1500
param set RC3_MAX 2003
param set RC3_MIN 992
param set RC3_TRIM 992
param set RC4_MAX 2011
param set RC4_MIN 997
param set RC4_TRIM 1504
param set RC4_REV -1
param set RC6_MAX 2016
param set RC6_MIN 992
param set RC6_TRIM 1504
param set RC_CHAN_CNT 8
param set RC_MAP_MODE_SW 5
param set RC_MAP_POSCTL_SW 7
param set RC_MAP_RETURN_SW 8
param set MC_PITCHRATE_P 0.05
param set MC_ROLLRATE_P 0.05
mixer load /dev/pwm_output0 ../../ROMFS/px4fmu_common/mixers/quad_x.main.mix
```
