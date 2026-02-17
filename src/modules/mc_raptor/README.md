# RAPTOR


## SITL
#### Standalone Usage (Without External Trajectory Setpoint)
Build PX4 SITL with Raptor, disable QGC requirement, and adjust the `IMU_GYRO_RATEMAX` to match the simulation IMU rate
```bash
make px4_sitl_raptor gz_x500
param set NAV_DLL_ACT 0
param set COM_DISARM_LAND -1 # When taking off in offboard the landing detector can cause mid-air disarms
param set IMU_GYRO_RATEMAX 250 # Just for SITL. Tested with IMU_GYRO_RATEMAX=400 on real FCUs
param set MC_RAPTOR_ENABLE 1
param set MC_RAPTOR_OFFB 0
param save
```
Upload the RAPTOR checkpoint to the "SD card": Separate terminal
```bash
mavproxy.py --master udp:127.0.0.1:14540
ftp mkdir /raptor # for the real FMU use: /fs/microsd/raptor
ftp put src/modules/mc_raptor/blob/policy.tar /raptor/policy.tar
```
restart (ctrl+c)
```bash
make px4_sitl_raptor gz_x500
commander takeoff
commander status
```

Note the external mode ID of `RAPTOR` in the status report

```bash
commander mode ext{RAPTOR_MODE_ID}
```


#### Usage with External Trajectory Setpoint


Send Lissajous setpoints via Mavlink:
```bash
pip install px4
px4 udp:localhost:14540 track lissajous --A 2.0 --B 1.0 --duration 10 --ramp-duration 5 --takeoff 10.0 --iterations 2
```


## SIH
```bash
make px4_fmu-v6c_raptor upload
```
In QGroundControl:
- Airframes => SIH Quadrotor X
- Settings => Comm Links => Disable Pixhawk (disable automatic USB serial connection)
```bash
mavproxy.py --master /dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 --out udp:localhost:14550 --out udp:localhost:13337 --out udp:localhost:13338
```
New terminal (optional):
```bash
./Tools/mavlink_shell.py udp:localhost:13338
```

```bash
param set SIH_IXX 0.005
param set SIH_IYY 0.005
param set SIH_IZZ 0.010
param set IMU_GYRO_RATEMAX 400
param save
reboot
```

New terminal:
```bash
./Tools/simulation/jmavsim/jmavsim_run.sh -u -p 13337 -o
```


## Real World
Using a DroneBridge WiFi telemetry @ 1000000 baud (also set `SER_TEL1_BAUD=1000000`) and maximum packet size = 16. It seems like larger maximum packet sizes can lead to delays in forwarding the `SET_POSITION_TARGET_LOCAL_NED` messages to `trajectory_setpoint`.
```bash
./Tools/mavlink_shell.py tcp:192.168.2.1:5760
```
```bash
px4 tcp:192.168.2.1:5760 track lissajous --A 0.5 --B 0.5 --duration 10 --ramp-duration 5 --takeoff 1.0 --iterations 2
```


## Troubleshooting


```bash
cat > logger_topics.txt << EOF
raptor_status 0
raptor_input 0
trajectory_setpoint 0
vehicle_local_position 0
vehicle_angular_velocity 0
vehicle_attitude 0
vehicle_status 0
actuator_motors 0
EOF
```
```bash
mavproxy.py
```
```bash
ftp mkdir /fs/microsd/etc
ftp mkdir /fs/microsd/etc/logging
ftp put logger_topics.txt /fs/microsd/etc/logging/logger_topics.txt
```

For SITL:

```bash
ftp mkdir etc
ftp mkdir logging
ftp put logger_topics.txt etc/logging/logger_topics.txt
```
