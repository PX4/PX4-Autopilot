# VOXL 2 barometer temperature calibration

The process to calibrate the barometer on VOXL 2 and VOXL 2 mini
involves cooling the board, then running through a data collection
phase while the board is heating up to capture the effects of temperature
change on the reported pressure while the board is at a fixed altitude. Then, the data
is processed to determine the proper temperature compensation parameters
that are used by the `temperature_compensation` module in PX4.

## Instructions

* Cool the board.
    * For example: Turn off power and blow a fan on the board.
* Turn on the board, verify that px4 is running, and then run the `baro_temp_cal` script to
  calibrate the barometer.
* Wait for the calibration process to complete.
* It will output the parameter settings needed for the temperature compensation
  module into `/data/px4/param/parameters_baro_tc.cal`
* Set the parameters in PX4.
    * For example: `voxl-configure-px4-params -n -f /data/px4/param/parameters_baro_tc.cal`
* Reboot the board.

