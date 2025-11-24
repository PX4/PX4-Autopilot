#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

"""
Performs a composite analysis of ekf log analysis meta data for all .ulg.csv files in the specified directory
Generates and saves histogram plots for the meta data in population_data.pdf
Generates and saves population summary data in population_data.csv
"""

parser = argparse.ArgumentParser(description='Perform a composite analysis of ekf log analysis meta data for all .ulg.csv files in the specified directory')
parser.add_argument("directory_path")

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()
metadata_directory = args.directory_path

# Run the metadata analsyis tool to generate population statistics
# Loop through the csv files in the directory and load the metadata into a nested dictionary
print("\n"+"analysing all .ulog.csv files in "+metadata_directory)
population_data = {}
for filename in os.listdir(metadata_directory):
    if filename.endswith(".mdat.csv"):
        print("loading "+filename)
        # get the dictionary of fail and warning test thresholds from a csv file
        file = open(metadata_directory+"/"+filename)
        single_log_data = { } # meta data dictionary for a single log
        for line in file:
            x = line.split(",")
            a=x[0]
            b=x[1]
            c=x[2]
            try:
                single_log_data[a]=float(b)
            except:
                single_log_data[a]=b
        file.close()
        population_data[filename]=single_log_data

#        # print out the check levels
#        print('\n'+'The following metadata loaded from '+filename+' were used'+'\n')
#        val = population_data.get(filename, {}).get('imu_hfgyro_mean')
#        print(val)

# Open pdf file for plotting
from matplotlib.backends.backend_pdf import PdfPages
output_plot_filename = "population_data.pdf"
pp = PdfPages(metadata_directory+"/"+output_plot_filename)

# get statistics for the population
population_results = {
'master_warning_pct':[float('NaN'),'Percentage of logs with warnings'],
'master_fail_pct':[float('NaN'),'Percentage of logs with fails'],
'mag_warning_pct':[float('NaN'),'Percentage of logs with magnetometer sensor warnings'],
'mag_fail_pct':[float('NaN'),'Percentage of logs with magnetometer sensor fails'],
'yaw_warning_pct':[float('NaN'),'Percentage of logs with yaw sensor warnings'],
'yaw_fail_pct':[float('NaN'),'Percentage of logs with yaw sensor fails'],
'vel_warning_pct':[float('NaN'),'Percentage of logs with velocity sensor warnings'],
'vel_fail_pct':[float('NaN'),'Percentage of logs with velocity sensor fails'],
'pos_warning_pct':[float('NaN'),'Percentage of logs with position sensor warnings'],
'pos_fail_pct':[float('NaN'),'Percentage of logs with position sensor fails'],
'hgt_warning_pct':[float('NaN'),'Percentage of logs with height sensor warnings'],
'hgt_fail_pct':[float('NaN'),'Percentage of logs with height sensor fails'],
'hagl_warning_pct':[float('NaN'),'Percentage of logs with height above ground sensor warnings'],
'hagl_fail_pct':[float('NaN'),'Percentage of logs with height above ground sensor fails'],
'tas_warning_pct':[float('NaN'),'Percentage of logs with airspeed sensor warnings'],
'tas_fail_pct':[float('NaN'),'Percentage of logs with airspeed ground sensor fails'],
'mag_test_max_avg':[float('NaN'),'The mean of the maximum  in-flight values of the magnetic field sensor innovation consistency test ratio'],
'mag_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the magnetic field sensor innovation consistency test ratio'],
'vel_test_max_avg':[float('NaN'),'The mean of the maximum  in-flight values of the velocity sensor innovation consistency test ratio'],
'vel_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the velocity sensor innovation consistency test ratio'],
'pos_test_max_avg':[float('NaN'),'The mean of the maximum  in-flight values of the position sensor innovation consistency test ratio'],
'pos_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the position sensor innovation consistency test ratio'],
'hgt_test_max_avg':[float('NaN'),'The mean of the maximum  in-flight values of the height sensor innovation consistency test ratio'],
'hgt_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the height sensor innovation consistency test ratio'],
'tas_test_max_avg':[float('NaN'),'The mean of the maximum  in-flight values of the airspeed sensor innovation consistency test ratio'],
'tas_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the airspeed sensor innovation consistency test ratio'],
'hagl_test_max_avg':[float('NaN'),'The mean of the maximum in-flight values of the height above ground sensor innovation consistency test ratio'],
'hagl_test_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the height above ground sensor innovation consistency test ratio'],
'ofx_fail_pct_avg':[float('NaN'),'The mean percentage of innovation test fails for the X axis optical flow sensor'],
'ofy_fail_pct_avg':[float('NaN'),'The mean percentage of innovation test fails for the Y axis optical flow sensor'],
'imu_coning_max_avg':[float('NaN'),'The mean of the maximum in-flight values of the IMU delta angle coning vibration level (mrad)'],
'imu_coning_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the IMU delta angle coning vibration level (mrad)'],
'imu_hfgyro_max_avg':[float('NaN'),'The mean of the maximum in-flight values of the IMU high frequency gyro vibration level (rad/s)'],
'imu_hfgyro_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the IMU delta high frequency gyro vibration level (rad/s)'],
'imu_hfaccel_max_avg':[float('NaN'),'The mean of the maximum in-flight values of the IMU high frequency accel vibration level (m/s/s)'],
'imu_hfaccel_mean_avg':[float('NaN'),'The mean of the mean in-flight value of the IMU delta high frequency accel vibration level (m/s/s)'],
'obs_ang_median_avg':[float('NaN'),'The mean of the median in-flight value of the output observer angular tracking error magnitude (mrad)'],
'obs_vel_median_avg':[float('NaN'),'The mean of the median in-flight value of the output observer velocity tracking error magnitude (m/s)'],
'obs_pos_median_avg':[float('NaN'),'The mean of the median in-flight value of the output observer position tracking error magnitude (m)'],
}

# get population summary statistics
found_keys = population_data.keys()

# master status
result = [population_data[k].get('master_status') for k in found_keys]
population_results['master_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['master_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# magnetometer sensor
result = [population_data[k].get('mag_sensor_status') for k in found_keys]
population_results['mag_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['mag_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# yaw sensor
result = [population_data[k].get('yaw_sensor_status') for k in found_keys]
population_results['yaw_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['yaw_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# velocity sensor
result = [population_data[k].get('vel_sensor_status') for k in found_keys]
population_results['vel_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['vel_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# position sensor
result = [population_data[k].get('pos_sensor_status') for k in found_keys]
population_results['pos_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['pos_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# height sensor
result = [population_data[k].get('hgt_sensor_status') for k in found_keys]
population_results['hgt_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['hgt_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# height above ground sensor
result = [population_data[k].get('hagl_sensor_status') for k in found_keys]
population_results['hagl_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['hagl_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# height above ground sensor
result = [population_data[k].get('tas_sensor_status') for k in found_keys]
population_results['tas_warning_pct'][0] = 100.0 * result.count('Warning') / len(result)
population_results['tas_fail_pct'][0] = 100.0 * result.count('Fail') / len(result)

# Mean and max innovation test levels
# Magnetometer
temp = np.asarray([population_data[k].get('mag_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('mag_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['mag_test_max_avg'][0] = np.mean(result1)
    population_results['mag_test_mean_avg'][0] = np.mean(result2)

    plt.figure(1,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Magnetometer Innovation Test Ratio Maximum")
    plt.xlabel("mag_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Magnetometer Innovation Test Ratio Mean")
    plt.xlabel("mag_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(1)

# Velocity Sensor (GPS)
temp = np.asarray([population_data[k].get('vel_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('vel_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['vel_test_max_avg'][0] = np.mean(result1)
    population_results['vel_test_mean_avg'][0] = np.mean(result2)

    plt.figure(2,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Velocity Innovation Test Ratio Maximum")
    plt.xlabel("vel_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Velocity Innovation Test Ratio Mean")
    plt.xlabel("vel_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(2)

# Position Sensor (GPS or external vision)
temp = np.asarray([population_data[k].get('pos_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('pos_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['pos_test_max_avg'][0] = np.mean(result1)
    population_results['pos_test_mean_avg'][0] = np.mean(result2)

    plt.figure(3,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Position Innovation Test Ratio Maximum")
    plt.xlabel("pos_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Position Innovation Test Ratio Mean")
    plt.xlabel("pos_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(3)

# Height Sensor
temp = np.asarray([population_data[k].get('hgt_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('hgt_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['hgt_test_max_avg'][0] = np.mean(result1)
    population_results['hgt_test_mean_avg'][0] = np.mean(result2)

    plt.figure(4,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Height Innovation Test Ratio Maximum")
    plt.xlabel("pos_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Height Innovation Test Ratio Mean")
    plt.xlabel("pos_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(4)

# Airspeed Sensor
temp = np.asarray([population_data[k].get('tas_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('tas_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['tas_test_max_avg'][0] = np.mean(result1)
    population_results['tas_test_mean_avg'][0] = np.mean(result2)

    plt.figure(5,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Airspeed Innovation Test Ratio Maximum")
    plt.xlabel("tas_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Airspeed Innovation Test Ratio Mean")
    plt.xlabel("tas_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(5)

# Height Above Ground Sensor
temp = np.asarray([population_data[k].get('hagl_test_max') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('hagl_test_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['hagl_test_max_avg'][0] = np.mean(result1)
    population_results['hagl_test_mean_avg'][0] = np.mean(result2)

    plt.figure(6,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - HAGL Innovation Test Ratio Maximum")
    plt.xlabel("hagl_test_max")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - HAGL Innovation Test Ratio Mean")
    plt.xlabel("hagl_test_mean")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(6)

# Optical Flow Sensor
temp = np.asarray([population_data[k].get('ofx_fail_percentage') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('ofy_fail_percentage') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['ofx_fail_pct_avg'][0] = np.mean(result1)
    population_results['ofy_fail_pct_avg'][0] = np.mean(result2)

    plt.figure(7,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - Optical Flow X Axis Fail Percentage")
    plt.xlabel("ofx_fail_percentage")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - Optical Flow Y Axis Fail Percentage")
    plt.xlabel("ofy_fail_percentage")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(7)

# IMU coning vibration levels
temp = np.asarray([population_data[k].get('imu_coning_peak') for k in found_keys])
result1 = 1000.0 * temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('imu_coning_mean') for k in found_keys])
result2 = 1000.0 * temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['imu_coning_max_avg'][0] = np.mean(result1)
    population_results['imu_coning_mean_avg'][0] = np.mean(result2)

    plt.figure(8,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - IMU Coning Vibration Peak")
    plt.xlabel("imu_coning_max (mrad)")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - IMU Coning Vibration Mean")
    plt.xlabel("imu_coning_mean (mrad)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(8)

# IMU high frequency delta angle vibration levels
temp = np.asarray([population_data[k].get('imu_hfgyro_peak') for k in found_keys])
result1 = 1000.0 * temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('imu_hfgyro_mean') for k in found_keys])
result2 = 1000.0 * temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['imu_hfgyro_max_avg'][0] = np.mean(result1)
    population_results['imu_hfgyro_mean_avg'][0] = np.mean(result2)

    plt.figure(9,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - IMU HF Gyroscope Vibration Peak")
    plt.xlabel("imu_hfgyro_max (rad/s)")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - IMU HF Gyroscope Vibration Mean")
    plt.xlabel("imu_hfgyro_mean (rad/s)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(9)

# IMU high frequency accel vibration levels
temp = np.asarray([population_data[k].get('imu_hfaccel_peak') for k in found_keys])
result1 = temp[np.isfinite(temp)]
temp = np.asarray([population_data[k].get('imu_hfaccel_mean') for k in found_keys])
result2 = temp[np.isfinite(temp)]

if (len(result1) > 0 and len(result2) > 0):
    population_results['imu_hfaccel_max_avg'][0] = np.mean(result1)
    population_results['imu_hfaccel_mean_avg'][0] = np.mean(result2)

    plt.figure(10,figsize=(20,13))

    plt.subplot(2,1,1)
    plt.hist(result1)
    plt.title("Gaussian Histogram - IMU HF Accelerometer Vibration Peak")
    plt.xlabel("imu_hfaccel_max (m/s/s)")
    plt.ylabel("Frequency")

    plt.subplot(2,1,2)
    plt.hist(result2)
    plt.title("Gaussian Histogram - IMU HF Accelerometer Vibration Mean")
    plt.xlabel("imu_hfaccel_mean (m/s/s)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(10)

# Output Observer Angular Tracking
temp = np.asarray([population_data[k].get('output_obs_ang_err_median') for k in found_keys])
result = 1000.0 * temp[np.isfinite(temp)]

if (len(result) > 0):
    population_results['obs_ang_median_avg'][0] = np.mean(result)

    plt.figure(11,figsize=(20,13))

    plt.hist(result)
    plt.title("Gaussian Histogram - Output Observer Angular Tracking Error Median")
    plt.xlabel("output_obs_ang_err_median (mrad)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(11)

# Output Observer Velocity Tracking
temp = np.asarray([population_data[k].get('output_obs_vel_err_median') for k in found_keys])
result = temp[np.isfinite(temp)]

if (len(result) > 0):
    population_results['obs_vel_median_avg'][0] = np.mean(result)

    plt.figure(12,figsize=(20,13))

    plt.hist(result)
    plt.title("Gaussian Histogram - Output Observer Velocity Tracking Error Median")
    plt.xlabel("output_obs_ang_err_median (m/s)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(12)

# Output Observer Position Tracking
temp = np.asarray([population_data[k].get('output_obs_pos_err_median') for k in found_keys])
result = temp[np.isfinite(temp)]

if (len(result) > 0):
    population_results['obs_pos_median_avg'][0] = np.mean(result)

    plt.figure(13,figsize=(20,13))

    plt.hist(result)
    plt.title("Gaussian Histogram - Output Observer Position Tracking Error Median")
    plt.xlabel("output_obs_ang_err_median (m)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(13)

# IMU delta angle bias
temp = np.asarray([population_data[k].get('imu_dang_bias_median') for k in found_keys])
result = temp[np.isfinite(temp)]

if (len(result) > 0):
    plt.figure(14,figsize=(20,13))

    plt.hist(result)
    plt.title("Gaussian Histogram - IMU Delta Angle Bias Median")
    plt.xlabel("imu_dang_bias_median (rad)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(14)

# IMU delta velocity bias
temp = np.asarray([population_data[k].get('imu_dvel_bias_median') for k in found_keys])
result = temp[np.isfinite(temp)]

if (len(result) > 0):
    plt.figure(15,figsize=(20,13))

    plt.hist(result)
    plt.title("Gaussian Histogram - IMU Delta Velocity Bias Median")
    plt.xlabel("imu_dvel_bias_median (m/s)")
    plt.ylabel("Frequency")

    pp.savefig()
    plt.close(15)

# close the pdf file
pp.close()
print('Population summary plots saved in population_data.pdf')

# don't display to screen
#plt.show()

# clase all figures
plt.close("all")

# write metadata to a .csv file
population_results_filename = metadata_directory + "/population_data.csv"
file = open(population_results_filename,"w")

file.write("name,value,description\n")

# loop through the dictionary and write each entry on a separate row, with data comma separated
# save data in alphabetical order
key_list = list(population_results.keys())
key_list.sort()
for key in key_list:
    file.write(key+","+str(population_results[key][0])+","+population_results[key][1]+"\n")

file.close()

print('Population summary data saved in population_data.csv')

single_log_results = {
'filter_faults_max':[float('NaN'),'Largest recorded value of the filter internal fault bitmask. Should always be zero.'],
'hagl_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the height above ground sensor innovation consistency test.'],
'hagl_percentage_amber':[float('NaN'),'The percentage of in-flight height above ground sensor innovation consistency test values > 0.5.'],
'hagl_percentage_red':[float('NaN'),'The percentage of in-flight height above ground sensor innovation consistency test values > 1.0.'],
'hagl_sensor_status':['Pass','Height above ground sensor check summary. This sensor data is normally sourced from a rangefinder sensor. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'hagl_test_max':[float('NaN'),'The maximum in-flight value of the height above ground sensor innovation consistency test ratio.'],
'hagl_test_mean':[float('NaN'),'The mean in-flight value of the height above ground sensor innovation consistency test ratio.'],
'hgt_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the height sensor innovation consistency test.'],
'hgt_percentage_amber':[float('NaN'),'The percentage of in-flight height sensor innovation consistency test values > 0.5.'],
'hgt_percentage_red':[float('NaN'),'The percentage of in-flight height sensor innovation consistency test values > 1.0.'],
'hgt_sensor_status':['Pass','Height sensor check summary. This sensor data can be sourced from either Baro, GPS, range fidner or external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'hgt_test_max':[float('NaN'),'The maximum in-flight value of the height sensor innovation consistency test ratio.'],
'hgt_test_mean':[float('NaN'),'The mean in-flight value of the height sensor innovation consistency test ratio.'],
'imu_coning_mean':[float('NaN'),'Mean in-flight value of the IMU delta angle coning vibration metric (rad^2)'],
'imu_coning_peak':[float('NaN'),'Peak in-flight value of the IMU delta angle coning vibration metric (rad^2)'],
'imu_hfgyro_mean':[float('NaN'),'Mean in-flight value of the IMU gyro high frequency vibration metric (rad/s)'],
'imu_hfgyro_peak':[float('NaN'),'Peak in-flight value of the IMU gyro high frequency vibration metric (rad/s)'],
'imu_hfaccel_mean':[float('NaN'),'Mean in-flight value of the IMU accel high frequency vibration metric (m/s/s)'],
'imu_hfaccel_peak':[float('NaN'),'Peak in-flight value of the IMU accel high frequency vibration metric (m/s/s)'],
'imu_sensor_status':['Pass','IMU sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'in_air_transition_time':[float('NaN'),'The time in seconds measured from startup that the EKF transtioned into in-air mode. Set to a nan if a transition event is not detected.'],
'mag_percentage_amber':[float('NaN'),'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 0.5.'],
'mag_percentage_red':[float('NaN'),'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 1.0.'],
'mag_sensor_status':['Pass','Magnetometer sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'mag_test_max':[float('NaN'),'The maximum in-flight value of the magnetic field sensor innovation consistency test ratio.'],
'mag_test_mean':[float('NaN'),'The mean in-flight value of the magnetic field sensor innovation consistency test ratio.'],
'magx_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the X-axis magnetic field sensor innovation consistency test.'],
'magy_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the Y-axis magnetic field sensor innovation consistency test.'],
'magz_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the Z-axis magnetic field sensor innovation consistency test.'],
'master_status':['Pass','Master check status which can be either Pass Warning or Fail. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'ofx_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the optical flow sensor X-axis innovation consistency test.'],
'ofy_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the optical flow sensor Y-axis innovation consistency test.'],
'on_ground_transition_time':[float('NaN'),'The time in seconds measured from startup  that the EKF transitioned out of in-air mode. Set to a nan if a transition event is not detected.'],
'output_obs_ang_err_median':[float('NaN'),'Median in-flight value of the output observer angular error (rad)'],
'output_obs_pos_err_median':[float('NaN'),'Median in-flight value of the output observer position error (m)'],
'output_obs_vel_err_median':[float('NaN'),'Median in-flight value of the output observer velocity error (m/s)'],
'pos_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
'pos_percentage_amber':[float('NaN'),'The percentage of in-flight position sensor consolidated innovation consistency test values > 0.5.'],
'pos_percentage_red':[float('NaN'),'The percentage of in-flight position sensor consolidated innovation consistency test values > 1.0.'],
'pos_sensor_status':['Pass','Position sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'pos_test_max':[float('NaN'),'The maximum in-flight value of the position sensor consolidated innovation consistency test ratio.'],
'pos_test_mean':[float('NaN'),'The mean in-flight value of the position sensor consolidated innovation consistency test ratio.'],
'tas_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the airspeed sensor innovation consistency test.'],
'tas_percentage_amber':[float('NaN'),'The percentage of in-flight airspeed sensor innovation consistency test values > 0.5.'],
'tas_percentage_red':[float('NaN'),'The percentage of in-flight airspeed sensor innovation consistency test values > 1.0.'],
'tas_sensor_status':['Pass','Airspeed sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'tas_test_max':[float('NaN'),'The maximum in-flight value of the airspeed sensor innovation consistency test ratio.'],
'tas_test_mean':[float('NaN'),'The mean in-flight value of the airspeed sensor innovation consistency test ratio.'],
'tilt_align_time':[float('NaN'),'The time in seconds measured from startup that the EKF completed the tilt alignment. A nan value indicates that the alignment had completed before logging started or alignment did not complete.'],
'vel_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
'vel_percentage_amber':[float('NaN'),'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 0.5.'],
'vel_percentage_red':[float('NaN'),'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 1.0.'],
'vel_sensor_status':['Pass','Velocity sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'vel_test_max':[float('NaN'),'The maximum in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
'vel_test_mean':[float('NaN'),'The mean in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
'yaw_align_time':[float('NaN'),'The time in seconds measured from startup that the EKF completed the yaw alignment.'],
'yaw_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the yaw sensor innovation consistency test.'],
'yaw_sensor_status':['Pass','Yaw sensor check summary. This sensor data can be sourced from the magnetometer or an external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
}
