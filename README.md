# PX4 Autopilot Extended to Fully-Actuated Multirotors

This is the repository for the PX4 autopilot with the enhanced ability to work with fully-actuated robots. It is based on the original PX4 firmware (current version branched from master on Jan 21, 2020) and supports all the functionality and airframes of the original PX4 in the same way. However, it additionally provides implementations of attitude strategies for fully-actuated multorotors that allow the 6-DoF flight. It includes the definitions for a hexarotor with all rotors tilted sideways and has been tested on several UAVs.

For more information and to access the publications, please visit [the AirLab's website](http://theairlab.org/fully-actuated).

To access the original PX4 autopilot repository and website, please visit [here](https://github.com/PX4/PX4-Autopilot) and [here](https://px4.io/).

### How to use the firmware

For general PX4 functionality, please visit [PX4 User Guide](https://docs.px4.io/master/en/) and [PX4 Developer Guide](https://dev.px4.io/master/en/).

To be able to compile the firmware, you need to first [setup your system](https://dev.px4.io/master/en/setup/config_initial.html) and [install the toolchain](https://dev.px4.io/master/en/setup/dev_env.html). Then clone this repository and all its submodules:

```
git clone https://github.com/castacks/PX4-fully-actuated.git --recursive
```

Test that everything works fine using the guide on [this page](https://dev.px4.io/master/en/setup/building_px4.html#first-build-using-the-jmavsim-simulator). 

The current tilted-hex airframe is defined to have 30 degrees of side tilt, similar to what is described in our papers. To change the geometry for your airframe, you can modify the definition of the tilted hex or add your own definition. To add the new definition file, please consult the PX4 guides above. To modify our airframe, you can find it [here](https://github.com/castacks/PX4-fully-actuated/blob/v1.10-master/src/lib/mixer/MultirotorMixer/geometries/hex_tilt_x.toml).

After compiling the firmware and uploading it on your hardware (see [here](https://dev.px4.io/master/en/setup/building_px4.html)), choose `Hexarotor x with tilted arms` airframe. This firmware also adds a set of parameters all starting with `OMNI_`. To see the description of the parameters, please refer [here](https://github.com/castacks/PX4-fully-actuated/blob/v1.10-master/src/modules/mc_pos_control/mc_pos_control_params.c). The most important parameter is `OMNI_ATT_MODE` which supports the attitude strategies. 

We have tested the firmware only with Pixracer. If you have flash memory issues when compiling for your hardware, one solution is to comment the geometries that you don't need in [this CMakeLists file](https://github.com/castacks/PX4-fully-actuated/blob/v1.10-master/src/lib/mixer/MultirotorMixer/CMakeLists.txt).

### Simulation and more

More detailed guide will be incrementally added here. Please reach us directly or submit an issue on this repository if you have specific questions, concerns, feedback or bug reports.

The files for the Gazebo simulation model and detailed guide on compiling for SITL will be added here by the end of November 2020.

### Disclaimer

Note that changing any of the parameters (e.g., the attitude modes) will affect the flight behavior of the UAV mid-flight. 

Finally, we are still in the testing/development phase and the code is in no way fully tested or complete. Use the code at your own risk and please report any bugs or issues to help us make the firmware more useful for the users.

### Acknowledgment

In the initial commits, the idea of how to extend PX4 to support 3-D thrust and fully-actuated vehicles have been taken from [here](https://github.com/jlecoeur/Firmware/pull/1).
