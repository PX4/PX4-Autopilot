* [x] Build headless SITL
* [x] How to choose plane/quad? Or is its a runtime config, not build time?
```bash
cd /home/ayakuba/src/poc/px4/build/px4_sitl_sih/src/modules/simulation/simulator_sih
/usr/bin/cmake -E env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim
/home/ayakuba/src/poc/px4/build/px4_sitl_sih/bin/px4
```
* [x] Ensure SITL has all sensors (GPS, Acc, Gyro, Baro)
* [x] Expose 2 ports in SITL: `14550, 14540`
* [x] Ensure Beacon support is enabled
* [ ] How to provide initial params?
* [x] Connect Copilot: `./copilot --mav udpin:0.0.0.0:14540`
* [x] Configure PX4 to use beacon ranges: `EKF2_RNGBC_CTRL, 1`
* [x] Debug SITL
* [x] Disable GPS: EKF2_GPS_CTRL=0
* [x] Debug build: export PX4_CMAKE_BUILD_TYPE=Debug
* [x] SITL starting location: export PX4_HOME_LAT(LON/ALT)=49.796766
* [ ] Clean SITL start with default params:
	Delete build/px4_sitl_sih/rootfs/parameters.bson?
