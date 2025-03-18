# Modules Reference: Camera (Driver)
## camera_trigger
Source: [drivers/camera_trigger](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_trigger)


### Description

Camera trigger driver.

This module triggers cameras that are connected to the flight-controller outputs,
or simple MAVLink cameras that implement the MAVLink trigger protocol.

The driver responds to the following MAVLink trigger commands being found in missions or recieved over MAVLink:

- `MAV_CMD_DO_TRIGGER_CONTROL`
- `MAV_CMD_DO_DIGICAM_CONTROL`
- `MAV_CMD_DO_SET_CAM_TRIGG_DIST`
- `MAV_CMD_OBLIQUE_SURVEY`

The commands cause the driver to trigger camera image capture based on time or distance.
Each time an image capture is triggered, the `CAMERA_TRIGGER` MAVLink message is emitted.

A "simple MAVLink camera" is one that supports the above command set.
When configured for this kind of camera, all the driver does is emit the `CAMERA_TRIGGER` MAVLink message as expected.
The incoming commands must be forwarded to the MAVLink camera, and are automatically emitted to MAVLink channels
when found in missions.

The driver is configured using [Camera Trigger parameters](../advanced_config/parameter_reference.md#camera-trigger).
In particular:

- `TRIG_INTERFACE` - How the camera is connected to flight controller (PWM, GPIO, Seagull, MAVLink)
- `TRIG_MODE` - Distance or time based triggering, with values set by `TRIG_DISTANCE` and `TRIG_INTERVAL`.

[Setup/usage information](../camera/index.md).

<a id="camera_trigger_usage"></a>
### Usage
```
camera_trigger <command> [arguments...]
 Commands:
   start

   stop          Stop driver

   status        Print driver status information

   test          Trigger one image (not logged or forwarded to GCS)

   test_power    Toggle power
```
