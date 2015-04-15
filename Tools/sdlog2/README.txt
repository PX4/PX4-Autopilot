====== PX4 LOG CONVERSION ======

On each log session (commonly started and stopped by arming and disarming the vehicle) a new file logxxx.bin is created. In many cases there will be only one logfile named log001.bin (only one flight).

There are two conversion scripts in this ZIP file:

logconv.m: This is a MATLAB script which will automatically convert and display the flight data with a GUI. If running this script, the second script can be ignored.

sdlog2_dump.py: This is a Python script (compatible with v2 and v3) which converts the self-describing binary log format to a CSV file. To export a CSV file from within a shell (Windows CMD or BASH on Linux / Mac OS), run:

python sdlog2_dump.py log001.bin -f "export.csv" -t "TIME" -d "," -n ""

Python can be downloaded from http://python.org, but is available as default on Mac OS and Linux.