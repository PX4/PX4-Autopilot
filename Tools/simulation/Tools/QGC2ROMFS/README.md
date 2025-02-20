## Purpose

The idea of this tool is to automate the writing of the gazebo parameters by automatizing the passage from the QGC parameters to the gazebo parameters as well as create the necessary files to run a simulation.

## Setup

In order to run this tool, it is possible to use an semi-automated method or a manual method to do so follow these steps:

### Run

1. Open QGC and obtain the parameters of the vehicle to mimic in simulation.
2. Run `python3 run.py --params_file <QGC_parameters_file.params>`
3. Place the output in the simulated plane under (PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes) and added it in the CMakeList.txt
4. Run a simulation of the given vehicle and copy the errors to the errors.txt file (not all parameters are implemented in gz compared with the QGC so: 'ERROR [param] Parameter PARAM_NAME not found.' is an expected output from gazebo)
5. Run `python3 clean.py <romfs_path> <error_path>` (this will comment out the parameters that were giving as errors)
6. Run `python3 file_checker.py <romfs_filename>` (this will check for any double definitions)
6. Place the new output under (PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes)

### Automated

1. Open QGC and obtain the parameters of the vehicle to mimic in simulation.
2. Run `python3 run.py --params_file <QGC_parameters_file.params>`

## Functionality

The tool first passes the parameters from the QGC format `1	1	ASPD_BETA_GATE	1	6` to the gazebo format `param set-default ASPD_BETA_GATE 1.0`, this file is appended to a copy of --romfs_file, its named is based on the given `QGC_parameters_file.params` and its placed under the `--romfs_file` location (assumed to be the PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes), if it allready existed it will be replacing the existing file otherwise it created and added to the CMakeList.txt, a gazebo simulation is ran using a bash script ´make_px4_script.sh´ from which the errors are taken, since not all px4 parameters are defined in gazebo, with this errors.txt file `clean.py` is ran commenting the parameters that are either on the error.txt file or in conflict with simulation parameters for example `SENS_EN_GPSSIM`, Since the GPS is simulated, any offsets calibrated for the real vehicle should not be applied in the simulation, lastly `file_checker.py` is ran to try to find any possible double definitions of parameters.

## Usability

The current implementation provides a minimal working example. Note that this example is made to use PX4's Advanced Plane as a template and some manual ajustments are still required/encouraged since for example the number of control surfaced differs between the .param file and the gazebo Advanced Plane model

## IMPORTANT POINTS TO NOTE

- The simulation parameters being commented by clean.py because of simulated sensors ex. `SENS_EN_GPSSIM` made not be all the necessary or more than the necessary.
- If `parameter about the angle of pixhawk` is defined this will be changed to 0 deg since the simulation does not behave correctly with any other orientation

## Parameter Assignment

Below is a comprehensive list of the parameters that are commented based on the simulated sensors

|SENS_EN_GPSSIM ||||
|-|-|-|-|
|CAL_ACC0_ID|CAL_ACC0_PRIO|CAL_ACC0_ROT |CAL_ACC0_XOFF|
|CAL_ACC0_XSCALE|CAL_ACC0_YOFF| CAL_ACC0_YSCALE| CAL_ACC0_ZOFF|
|CAL_ACC0_ZSCALE|CAL_ACC1_ID|CAL_ACC1_PRIO |CAL_ACC1_ROT|
|CAL_ACC1_XOFF|CAL_ACC1_XSCALE|CAL_ACC1_YOFF |CAL_ACC1_YSCALE|
|CAL_ACC1_ZOFF|CAL_ACC1_ZSCALE|CAL_ACC2_ID |CAL_ACC3_ID|
|CAL_AIR_CMODEL|CAL_AIR_TUBED_MM|CAL_AIR_TUBELEN |CAL_BARO0_ID|
|CAL_BARO1_ID|CAL_BARO2_ID|CAL_BARO3_ID |CAL_GYRO0_ID|
|CAL_GYRO0_PRIO|CAL_GYRO0_ROT|CAL_GYRO0_XOFF |CAL_GYRO0_YOFF|
|CAL_GYRO0_ZOFF|CAL_GYRO1_ID|CAL_GYRO1_PRIO |CAL_GYRO1_ROT|
|CAL_GYRO1_XOFF|CAL_GYRO1_YOFF|CAL_GYRO1_ZOFF |CAL_GYRO2_ID|
|CAL_GYRO3_ID|

|SENS_EN_MAGSIM ||||
|-|-|-|-|
|CAL_MAG0_ID|CAL_MAG0_PRIO|CAL_MAG0_ROT |CAL_MAG0_XCOMP|
|CAL_MAG0_XODIAG|CAL_MAG0_XOFF| CAL_MAG0_XSCALE| CAL_MAG0_YCOMP|
|CAL_MAG0_YODIAG|CAL_MAG0_YOFF|CAL_MAG0_YSCALE |CAL_MAG0_ZCOMP|
|CAL_MAG0_ZODIAG|CAL_MAG0_ZOFF|CAL_MAG0_ZSCALE |CAL_MAG1_ID|
|CAL_MAG1_PRIO|CAL_MAG1_ROT|CAL_MAG1_XCOMP |CAL_MAG1_XODIAG|
|CAL_MAG1_XOFF|CAL_MAG1_XSCALE|CAL_MAG1_YCOMP |CAL_MAG1_YODIAG|
|CAL_MAG1_YOFF|CAL_MAG1_YSCALE|CAL_MAG1_ZCOMP |CAL_MAG1_ZODIAG|
|CAL_MAG1_ZOFF|CAL_MAG1_ZSCALE|CAL_MAG2_ID |CAL_MAG2_ROT|
|CAL_MAG3_ID|CAL_MAG3_ROT|CAL_MAG_COMP_TYP |CAL_MAG_SIDES|

|SENS_EN_ARSPDSIM ||||
|-|-|-|-|
|ASPD_BETA_GATE|ASPD_BETA_NOISE|ASPD_DO_CHECKS |ASPD_FALLBACK_GW|
|ASPD_FS_INNOV|ASPD_FS_INTEG| ASPD_FS_T_START| ASPD_FS_T_STOP|
|ASPD_PRIMARY|ASPD_SCALE_1|ASPD_SCALE_2 |ASPD_SCALE_3|
|ASPD_SCALE_APPLY|ASPD_SCALE_NSD|ASPD_TAS_GATE |ASPD_TAS_NOISE|
|ASPD_WERR_THR|ASPD_WIND_NSD|

## Future Work

The tool, while self-contained, could be expanded into multiple directions.

1. The make px4_sitl gz_<vehicle> should be closed upon completion of the errors check, since its just a step of the process.
2. A better file_checker.py can be written to check for example a diference between QGC and gz control surfaces.
