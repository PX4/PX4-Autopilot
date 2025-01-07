# VOXL 2 barometer temperature calibration

The process to calibrate the barometer on VOXL 2 and VOXL 2 mini
involves cooling the board, then running through a data collection
phase while the board is heating up to capture the effects of temperature
change on the reported pressure while the board is at a fixed altitude. Then, the data captured
can be processed to determine the proper temperature compensation parameters
that are used by the `temperature_compensation` module as PX4 is running

## Data capture

* Copy the data capture script (`boards/modalai/voxl2/scripts/plot_px4_baro.py`)
  to the board
* On the board install dependencies for the script (Must be connected to Internet)
    * `pip3 install plotly psutil --upgrade`
* Cool the board. For example, turn off power and blow a fan on the board.
* Turn on the board, verify that px4 is running, and then run the script to
  capture the data. For example: `python3 plot_px4_baro.py -n 1250 -i 1 -c 4 -p baro_test0`
    * The `-n` option tells it how many samples to gather
    * The `-i` option tells it how many times to run the test (iterations)
    * The `-c` option tells it how many cores to use for the CPU loading. This
      runs the Linux stress command on that many cores to cause more heat to be
      generated
* Wait for the capture process to complete

## Data processing

* Copy the data capture script (`boards/modalai/voxl2/scripts/process_baro_logs.py`)
  to the board
* The data capture process will have created a new directory named `baro_test0`. This
  directory contains the data for this processing step
* Run the processing script. For example: `python3 process_baro_logs.py`
    * Note, the script must be run where the `baro_test0` subdirectory is located in order
      to find the results from the data capture step
* It will output the parameter settings needed for the temperature compensation
  module
* Set the parameters in PX4, reboot the board, run a test flight

