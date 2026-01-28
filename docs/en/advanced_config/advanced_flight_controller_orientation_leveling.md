# Advanced Flight Controller Orientation Tuning

These instructions can be used to manually fine-tune the orientation and horizon level, for example to correct for small sensor board misalignments or minor calibration errors.

If there is a persistent drift bias (often seen in multirotors but not limited to them), it is a good strategy to trim it with the help of these fine-tuning offset angle parameters, instead of using the trimmers of your RC Transmitter.
This ensures the vehicle will maintain the trimming when in fully autonomous flight.

::: info
These instructions are "advanced", and not recommended for regular users (the broad tuning is generally sufficient).
:::

## Setting Orientation Parameters

The [SENS_BOARD_ROT](../advanced_config/parameter_reference.md#SENS_BOARD_ROT) parameter defines the rotation of the flight controller board relative to the vehicle frame, while the fine tuning offsets ([SENS_BOARD_X_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_X_OFF), [SENS_BOARD_Y_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_Y_OFF), [SENS_BOARD_Z_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_Z_OFF)) set the rotation of the sensors relative to the board itself.
The fine tuning offsets are added to the `SENS_BOARD_ROT` angle in order to determine the total offset angles for the Yaw, Pitch and Roll orientation of the flight controller.

First perform the normal calibration for [Flight Controller Orientation](../config/flight_controller_orientation.md) and [Level Horizon Calibration](../config/level_horizon_calibration.md) to set the [SENS_BOARD_ROT](../advanced_config/parameter_reference.md#SENS_BOARD_ROT) parameter.

The other parameters can then be set in order to fine-tune the orientation of the IMU sensors relative to the board itself.

You can locate the parameters in QGroundControl as shown below:

1. Open QGroundControl menu: **Settings > Parameters > Sensor Calibration**.
1. The parameters as located in the section as shown below (or you can search for them):

   ![FC Orientation QGC v2](../../assets/qgc/setup/sensor/fc_orientation_qgc_v2.png)

## Parameter Summary

- [SENS_BOARD_ROT](../advanced_config/parameter_reference.md#SENS_BOARD_ROT): Rotation of the FMU board relative to the vehicle frame.
- [SENS_BOARD_X_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_X_OFF): Rotation, in degrees, around PX4FMU's X axis or Roll axis.
  Positive angles increase in CCW direction, negative angles increase in CW direction.
- [SENS_BOARD_Y_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_Y_OFF): Rotation, in degrees, around PX4FMU's Y axis or Pitch axis.
  Positive angles increase in CCW direction, negative angles increase in CW direction.
- [SENS_BOARD_Z_OFF](../advanced_config/parameter_reference.md#SENS_BOARD_Z_OFF): Rotation, in degrees, around PX4FMU's Z axis Yaw axis.
  Positive angles increase in CCW direction, negative angles increase in CW direction.
  
