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
* [x] How to provide initial params?
	ROMFS/px4fmu_common/init.d-posix/airframes/10046_navput_quadx
	param set-default <name> <value>
* [x] Connect Copilot: `./copilot --mav udpin:0.0.0.0:14540`
* [x] Configure PX4 to use beacon ranges: `EKF2_RNGBC_CTRL, 1`
* [x] Debug SITL
* [x] Disable GPS: EKF2_GPS_CTRL=0
* [x] Debug build: export PX4_CMAKE_BUILD_TYPE=Debug
* [x] SITL starting location: export PX4_HOME_LAT(LON/ALT)=49.796766
* [x] Clean SITL start with default params:
	rm -f build/px4_sitl_sih/rootfs/parameters.bson build/px4_sitl_sih/rootfs/parameters_backup.bson
	To always boot with your own predefined set (not just firmware defaults — e.g. keep EKF2_RNGBC_CTRL=1 and whatever else you're testing with), the cleanest approach is to snapshot a "golden" parameters.bson once you've got the params you want, then restore it before every launch instead of just deleting.


* [x] How to set initial position estimate? Why it is automagically set already?
	Its not. LocalPosition is tracked from abstact 0.0/0.0 origin.
	_ekf.resetGlobalPositionTo()
* [x] Add params to SIH to control Beacon simulation: enable/disable, max range, rate (interval)
* [x] Declare custom param group and forward to EKF for fusion control and tuning
* [x] Publish fusion state/EKF state - EKF2::PublishFusionControl() + PublishStatusFlags()
* [x] Aux Position source fusion - for initial estimate and recovery.
	Works out of the box. Good to proceed.
* [x] Add beacon ranges/innovation/pass to deBIN. Ensure that SIH fix - fixes fusion. DEBILKO!!!
* [ ] Add MLAT solver as AUX source.
  * [x] Separate fusion/data flow for SIH and NAVPUT.
  * [x] Expose AUX fusion status
  * [ ] Forecast ranges to the event horizon.
  * [x] Show AUX pos in the deBIN.
  * [ ] Looks like raw ranges require some pre-filtering, or higher noise ratio for AUX src.
  * [ ] Segfault. I suspect MLAT
	```
	./sitl_navput.bash: line 18: 304632 Segmentation fault      (core dumped) PX4_SIM_MODEL=navput_quadx PX4_SIMULATOR=sihsim ../bin/px
	```
* [ ] Check/add logic to set global origin from Aux source.
* [ ] Where does heading error between Navput and EK2 come from? It revolves counter-clock-wise always.
* [ ] Annoying timing exceptions during start when launching under debugger.
* [ ] Figure out how to publish to custom topic, but reuse existing messages.
* [x] Expose params to control fusion
