Instructions for running the EKF replay

1) Ensure the ‘EKF_replay’ directory is in a location you have full read and write access


2) Create a ‘TestData’ sub-directory under the ‘EKF_replay’ directory

A sample dataset can be downloaded here: https://drive.google.com/file/d/0By4v2BuLAaCfSW9fWl9aSWNGbGs/view?usp=sharing


3a) If replaying APM data:

Collect data with LOG_REPLAY = 1 and LOG_DISARMED = 1.
Convert data to a .mat file using the MissionPlanner ‘Create Matlab File’ option under the DataFlash Logs tab.
Convert .mat file to the required data format using the convert_apm_data.m script file. This will generate the following data files:

imu_data.mat
baro_data.mat
gps_data.mat
mag_data.mat

and optionally

rng_data.mat
flow_data.mat
viso_data.mat

Note: If the rangefinder, optical flow or ZED camera odometer data are not present in the log, then the corresponding sections in the convert_apm_data.m script file will need to be commented out.

Copy the generated .mat files into the /EKF_replay/TestData/APM directory.


3b) If replaying PX4 data:

Collect data with EK2_REC_RPL = 1
Convert the .ulg log file to .csv files using the PX4/pyulog python script https://github.com/PX4/pyulog/blob/master/pyulog/ulog2csv.py

Make this directory your current MARLAB working directory and fill these variables with the paths to the CSV files:
- sensors_file: path to *_sensor_combined_0.csv
- air_data_file: path to *vehicle_air_data_0.csv
- magnetometer_file: path to *_vehicle_magetometer_0.csv
- gps_file: path to *_vehicle_gps_position_0.csv

If a simulation was used, ground truth data can be converted as well:
- attitude_file: path to *_vehicle_attitude_groundtruth_0.csv
- localpos_file: path to *_vehicle_local_position_groundtruth_0.csv
- globalpos_file: path to *_vehicle_global_position_groundtruth_0.csv

The actuator controls can also be converted:
- actctrl_file: path to *_actuator_controls_0_0.csv
- actout_file: path to *_actuator_outputs_0.csv

When executing .../EKF_replay/px4_replay_import.m, the CSV files will be automatically loaded and converted using these scripts:
- .../EKF_replay/Common/convert_px4_sensor_combined_csv_data.m
  * uses imported data from:
    + sensors_file
    + air_data_file
    + magnetometer_file
  * generates:
    + imu_data.mat
    + baro_data.mat
    + mag_data.mat
- .../EKF_replay/Common/convert_px4_vehicle_gps_position_csv
  * uses imported data from:
    + gps_file
  * generates:
    + gps_data.mat
- .../EKF_replay/Common/convert_px4_groundtruth_csv_data
  * uses imported data from:
    + attitude_file
    + localpos_file
    + globalpos_file
  * generates:
    + attitude_data.mat
    + localpos_data.mat
    + globalpos_data.mat
- .../EKF_replay/Common/convert_px4_actuators_csv_data
  * uses imported data from:
    + actctrl_file
    + actout_file
  * generates:
    + actctrl_data.mat
    + actout_data.mat

If you have an optical flow and range finder sensor fitted, they must be imported and converted manually:

Import the .csv file containing the optical_flow_0 data into the matlab workspace and process it using …/EKF_replay/Common/convert_px4_optical_flow_csv_data.m.
Import the .csv file containing the distance_sensor_0 data into the matlab workspace and process it using …/EKF_replay/Common/convert_px4_distance_sensor_csv_data.m.
This will generate the following data files:

flow_data.mat
rng_data.mat


Finally copy the generated .mat files into the /EKF_replay/TestData/PX4 directory.


4) Make ‘…/EKF_replay/Filter’ the working directory.


5) Execute ‘SetParameterDefaults’ at the command prompt to load the default filter tuning parameter struct ‘param’ into the workspace. The defaults have been set to provide robust estimation across the entire data set, not optimised for accuracy.


6) Replay the data by running either the replay_apm_data.m, replay_px4_data.m or if you have px4 optical flow data, the replay_px4_optflow_data.m script file.

Output plots are saved as .png files in the ‘…/EKF_replay/OutputPlots/‘ directory.

Output data is written to the Matlab workspace in the ‘output’ struct.
