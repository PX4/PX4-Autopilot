# RAPTOR


## SITL
#### Standalone Usage (Without External Trajectory Setpoint)
Build PX4 SITL with Raptor, disable QGC requirement, and adjust the `IMU_GYRO_RATEMAX` to match the simulation IMU rate
```
make px4_sitl_raptor gz_x500
param set NAV_DLL_ACT 0
param set COM_DISARM_LAND -1 # When taking off in offboard the landing detector can cause mid-air disarms
param set IMU_GYRO_RATEMAX 250 # Just for SITL. Tested with IMU_GYRO_RATEMAX=400 on real FCUs
param set MC_RAPTOR_ENABLE 1
param set MC_RAPTOR_OFFB 0
```
Upload the RAPTOR checkpoint to the "SD card": Separate terminal
```
mavproxy.py --master udp:127.0.0.1:14540
ftp mkdir /raptor # for the real FMU use: /fs/microsd/raptor
ftp put policy.tar /raptor/policy.tar
```
restart (ctrl+c)
```
make px4_sitl_raptor gz_x500
commander takeoff
commander status
```

Note the external mode ID of `RAPTOR` in the status report

```
commander mode ext{RAPTOR_MODE_ID}
```


#### Usage with External Trajectory Setpoint


Send Lissajous setpoints via Mavlink:
```
pip install px4
px4 udp:localhost:14540 track lissajous --A 2.0 --B 1.0 --duration 10 --ramp-duration 5 --takeoff 10.0 --iterations 2
```


## SIH
```
make px4_fmu-v6c_raptor upload
```
In QGroundControl:
- Airframes => SIH Quadrotor X
- Settings => Comm Links => Disable Pixhawk (disable automatic USB serial connection)
```
mavproxy.py --master /dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 --out udp:localhost:14550 --out udp:localhost:13337 --out udp:localhost:13338
```
New terminal (optional):
```
./Tools/mavlink_shell.py udp:localhost:13338
```

```
param set SIH_IXX 0.005
param set SIH_IYY 0.005
param set SIH_IZZ 0.010
param set IMU_GYRO_RATEMAX 400
param save
reboot
```

New terminal:
```
./Tools/simulation/jmavsim/jmavsim_run.sh -u -p 13337 -o
```
