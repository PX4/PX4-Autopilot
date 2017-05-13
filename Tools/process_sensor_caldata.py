#! /usr/bin/env python

from __future__ import print_function

import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

from pyulog import *

"""
Reads in IMU data from a static thermal calibration test and performs a curve fit of gyro, accel and baro bias vs temperature
Data can be gathered using the following sequence:

1) Power up the board and set the TC_A_ENABLE, TC_B_ENABLE and TC_G_ENABLE parameters to 1
2) Set all CAL_GYR and CAL_ACC parameters to defaults
3) Set the SYS_LOGGER parameter to 1 to use the new system logger
4) Set the SDLOG_MODE parameter to 3 to enable logging of sensor data for calibration and power off
5) Cold soak the board for 30 minutes
6) Move to a warm dry, still air, constant pressure environment.
7) Apply power for 45 minutes, keeping the board still.
8) Remove power and extract the .ulog file
9) Open a terminal window in the Firmware/Tools directory and run the python calibration script script file: 'python process_sensor_caldata.py <full path name to .ulog file>
10) Power the board, connect QGC and load the parameter from the generated .params file onto the board using QGC. Due to the number of parameters, loading them may take some time.
11) TODO - we need a way for user to reliably tell when parameters have all been changed and saved.
12) After parameters have finished loading, set SDLOG_MODE to 1 to re-enable normal logging and remove power.
13) Power the board and perform a normal gyro and accelerometer sensor calibration using QGC. The board must be repowered after this step before flying due to large parameter changes and the thermal compensation parameters only being read on startup.

Outputs thermal compensation parameters in a file named <inputfilename>.params which can be loaded onto the board using QGroundControl
Outputs summary plots in a pdf file named <inputfilename>.pdf

"""

parser = argparse.ArgumentParser(description='Analyse the sensor_gyro  message data')
parser.add_argument('filename', metavar='file.ulg', help='ULog input file')

def is_valid_directory(parser, arg):
    if os.path.isdir(arg):
        # Directory exists so return the directory
        return arg
    else:
        parser.error('The directory {} does not exist'.format(arg))

args = parser.parse_args()
ulog_file_name = args.filename

ulog = ULog(ulog_file_name, None)
data = ulog.data_list

# extract gyro data
sensor_instance = 0
num_gyros = 0
for d in data:
    if d.name == 'sensor_gyro':
        if sensor_instance == 0:
            sensor_gyro_0 = d.data
            print('found gyro 0 data')
	    num_gyros = 1
        if sensor_instance == 1:
            sensor_gyro_1 = d.data
            print('found gyro 1 data')
	    num_gyros = 2
	if sensor_instance == 2:
            sensor_gyro_2 = d.data
            print('found gyro 2 data')
	    num_gyros = 3
	sensor_instance = sensor_instance +1

# extract accel data
sensor_instance = 0
num_accels = 0
for d in data:
    if d.name == 'sensor_accel':
        if sensor_instance == 0:
            sensor_accel_0 = d.data
            print('found accel 0 data')
	    num_accels = 1
        if sensor_instance == 1:
            sensor_accel_1 = d.data
            print('found accel 1 data')
	    num_accels = 2
        if sensor_instance == 2:
            sensor_accel_2 = d.data
            print('found accel 2 data')
	    num_accels = 3
        sensor_instance = sensor_instance +1

# extract baro data
sensor_instance = 0
for d in data:
    if d.name == 'sensor_baro':
        if sensor_instance == 0:
            sensor_baro_0 = d.data
            print('found baro 0 data')
        if sensor_instance == 1:
            sensor_baro_1 = d.data
            print('found baro 1 data')
        if sensor_instance == 2:
            sensor_baro_2 = d.data
            print('found baro 2 data')
        sensor_instance = sensor_instance +1

# open file to save plots to PDF
from matplotlib.backends.backend_pdf import PdfPages
output_plot_filename = ulog_file_name + ".pdf"
pp = PdfPages(output_plot_filename)

#################################################################################

# define data dictionary of gyro 0 thermal correction  parameters
gyro_0_params = {
'TC_G0_ID':0,
'TC_G0_TMIN':0.0,
'TC_G0_TMAX':0.0,
'TC_G0_TREF':0.0,
'TC_G0_X0_0':0.0,
'TC_G0_X1_0':0.0,
'TC_G0_X2_0':0.0,
'TC_G0_X3_0':0.0,
'TC_G0_X0_1':0.0,
'TC_G0_X1_1':0.0,
'TC_G0_X2_1':0.0,
'TC_G0_X3_1':0.0,
'TC_G0_X0_2':0.0,
'TC_G0_X1_2':0.0,
'TC_G0_X2_2':0.0,
'TC_G0_X3_2':0.0,
'TC_G0_SCL_0':1.0,
'TC_G0_SCL_1':1.0,
'TC_G0_SCL_2':1.0
}

# curve fit the data for gyro 0 corrections
if num_gyros >= 1:
	gyro_0_params['TC_G0_ID'] = int(np.median(sensor_gyro_0['device_id']))

	# find the min, max and reference temperature
	gyro_0_params['TC_G0_TMIN'] = np.amin(sensor_gyro_0['temperature'])
	gyro_0_params['TC_G0_TMAX'] = np.amax(sensor_gyro_0['temperature'])
	gyro_0_params['TC_G0_TREF'] = 0.5 * (gyro_0_params['TC_G0_TMIN'] + gyro_0_params['TC_G0_TMAX'])
	temp_rel = sensor_gyro_0['temperature'] - gyro_0_params['TC_G0_TREF']
	temp_rel_resample = np.linspace(gyro_0_params['TC_G0_TMIN']-gyro_0_params['TC_G0_TREF'], gyro_0_params['TC_G0_TMAX']-gyro_0_params['TC_G0_TREF'], 100)
	temp_resample = temp_rel_resample + gyro_0_params['TC_G0_TREF']

	# fit X axis
	coef_gyro_0_x = np.polyfit(temp_rel,sensor_gyro_0['x'],3)
	gyro_0_params['TC_G0_X3_0'] = coef_gyro_0_x[0]
	gyro_0_params['TC_G0_X2_0'] = coef_gyro_0_x[1]
	gyro_0_params['TC_G0_X1_0'] = coef_gyro_0_x[2]
	gyro_0_params['TC_G0_X0_0'] = coef_gyro_0_x[3]
	fit_coef_gyro_0_x = np.poly1d(coef_gyro_0_x)
	gyro_0_x_resample = fit_coef_gyro_0_x(temp_rel_resample)

	# fit Y axis
	coef_gyro_0_y = np.polyfit(temp_rel,sensor_gyro_0['y'],3)
	gyro_0_params['TC_G0_X3_1'] = coef_gyro_0_y[0]
	gyro_0_params['TC_G0_X2_1'] = coef_gyro_0_y[1]
	gyro_0_params['TC_G0_X1_1'] = coef_gyro_0_y[2]
	gyro_0_params['TC_G0_X0_1'] = coef_gyro_0_y[3]
	fit_coef_gyro_0_y = np.poly1d(coef_gyro_0_y)
	gyro_0_y_resample = fit_coef_gyro_0_y(temp_rel_resample)

	# fit Z axis
	coef_gyro_0_z = np.polyfit(temp_rel,sensor_gyro_0['z'],3)
	gyro_0_params['TC_G0_X3_2'] = coef_gyro_0_z[0]
	gyro_0_params['TC_G0_X2_2'] = coef_gyro_0_z[1]
	gyro_0_params['TC_G0_X1_2'] = coef_gyro_0_z[2]
	gyro_0_params['TC_G0_X0_2'] = coef_gyro_0_z[3]
	fit_coef_gyro_0_z = np.poly1d(coef_gyro_0_z)
	gyro_0_z_resample = fit_coef_gyro_0_z(temp_rel_resample)

	# gyro0 vs temperature
	plt.figure(1,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['x'],'b')
	plt.plot(temp_resample,gyro_0_x_resample,'r')
	plt.title('Gyro 0 Bias vs Temperature')
	plt.ylabel('X bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['y'],'b')
	plt.plot(temp_resample,gyro_0_y_resample,'r')
	plt.ylabel('Y bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_gyro_0['temperature'],sensor_gyro_0['z'],'b')
	plt.plot(temp_resample,gyro_0_z_resample,'r')
	plt.ylabel('Z bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of gyro 1 thermal correction  parameters
gyro_1_params = {
'TC_G1_ID':0,
'TC_G1_TMIN':0.0,
'TC_G1_TMAX':0.0,
'TC_G1_TREF':0.0,
'TC_G1_X0_0':0.0,
'TC_G1_X1_0':0.0,
'TC_G1_X2_0':0.0,
'TC_G1_X3_0':0.0,
'TC_G1_X0_1':0.0,
'TC_G1_X1_1':0.0,
'TC_G1_X2_1':0.0,
'TC_G1_X3_1':0.0,
'TC_G1_X0_2':0.0,
'TC_G1_X1_2':0.0,
'TC_G1_X2_2':0.0,
'TC_G1_X3_2':0.0,
'TC_G1_SCL_0':1.0,
'TC_G1_SCL_1':1.0,
'TC_G1_SCL_2':1.0
}

# curve fit the data for gyro 1 corrections
if num_gyros >= 2:
	gyro_1_params['TC_G1_ID'] = int(np.median(sensor_gyro_1['device_id']))

	# find the min, max and reference temperature
	gyro_1_params['TC_G1_TMIN'] = np.amin(sensor_gyro_1['temperature'])
	gyro_1_params['TC_G1_TMAX'] = np.amax(sensor_gyro_1['temperature'])
	gyro_1_params['TC_G1_TREF'] = 0.5 * (gyro_1_params['TC_G1_TMIN'] + gyro_1_params['TC_G1_TMAX'])
	temp_rel = sensor_gyro_1['temperature'] - gyro_1_params['TC_G1_TREF']
	temp_rel_resample = np.linspace(gyro_1_params['TC_G1_TMIN']-gyro_1_params['TC_G1_TREF'], gyro_1_params['TC_G1_TMAX']-gyro_1_params['TC_G1_TREF'], 100)
	temp_resample = temp_rel_resample + gyro_1_params['TC_G1_TREF']

	# fit X axis
	coef_gyro_1_x = np.polyfit(temp_rel,sensor_gyro_1['x'],3)
	gyro_1_params['TC_G1_X3_0'] = coef_gyro_1_x[0]
	gyro_1_params['TC_G1_X2_0'] = coef_gyro_1_x[1]
	gyro_1_params['TC_G1_X1_0'] = coef_gyro_1_x[2]
	gyro_1_params['TC_G1_X0_0'] = coef_gyro_1_x[3]
	fit_coef_gyro_1_x = np.poly1d(coef_gyro_1_x)
	gyro_1_x_resample = fit_coef_gyro_1_x(temp_rel_resample)

	# fit Y axis
	coef_gyro_1_y = np.polyfit(temp_rel,sensor_gyro_1['y'],3)
	gyro_1_params['TC_G1_X3_1'] = coef_gyro_1_y[0]
	gyro_1_params['TC_G1_X2_1'] = coef_gyro_1_y[1]
	gyro_1_params['TC_G1_X1_1'] = coef_gyro_1_y[2]
	gyro_1_params['TC_G1_X0_1'] = coef_gyro_1_y[3]
	fit_coef_gyro_1_y = np.poly1d(coef_gyro_1_y)
	gyro_1_y_resample = fit_coef_gyro_1_y(temp_rel_resample)

	# fit Z axis
	coef_gyro_1_z = np.polyfit(temp_rel,sensor_gyro_1['z'],3)
	gyro_1_params['TC_G1_X3_2'] = coef_gyro_1_z[0]
	gyro_1_params['TC_G1_X2_2'] = coef_gyro_1_z[1]
	gyro_1_params['TC_G1_X1_2'] = coef_gyro_1_z[2]
	gyro_1_params['TC_G1_X0_2'] = coef_gyro_1_z[3]
	fit_coef_gyro_1_z = np.poly1d(coef_gyro_1_z)
	gyro_1_z_resample = fit_coef_gyro_1_z(temp_rel_resample)

	# gyro1 vs temperature
	plt.figure(2,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['x'],'b')
	plt.plot(temp_resample,gyro_1_x_resample,'r')
	plt.title('Gyro 1 Bias vs Temperature')
	plt.ylabel('X bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['y'],'b')
	plt.plot(temp_resample,gyro_1_y_resample,'r')
	plt.ylabel('Y bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_gyro_1['temperature'],sensor_gyro_1['z'],'b')
	plt.plot(temp_resample,gyro_1_z_resample,'r')
	plt.ylabel('Z bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of gyro 2 thermal correction  parameters
gyro_2_params = {
'TC_G2_ID':0,
'TC_G2_TMIN':0.0,
'TC_G2_TMAX':0.0,
'TC_G2_TREF':0.0,
'TC_G2_X0_0':0.0,
'TC_G2_X1_0':0.0,
'TC_G2_X2_0':0.0,
'TC_G2_X3_0':0.0,
'TC_G2_X0_1':0.0,
'TC_G2_X1_1':0.0,
'TC_G2_X2_1':0.0,
'TC_G2_X3_1':0.0,
'TC_G2_X0_2':0.0,
'TC_G2_X1_2':0.0,
'TC_G2_X2_2':0.0,
'TC_G2_X3_2':0.0,
'TC_G2_SCL_0':1.0,
'TC_G2_SCL_1':1.0,
'TC_G2_SCL_2':1.0
}

# curve fit the data for gyro 2 corrections
if num_gyros >= 3:
	gyro_2_params['TC_G2_ID'] = int(np.median(sensor_gyro_2['device_id']))

	# find the min, max and reference temperature
	gyro_2_params['TC_G2_TMIN'] = np.amin(sensor_gyro_2['temperature'])
	gyro_2_params['TC_G2_TMAX'] = np.amax(sensor_gyro_2['temperature'])
	gyro_2_params['TC_G2_TREF'] = 0.5 * (gyro_2_params['TC_G2_TMIN'] + gyro_2_params['TC_G2_TMAX'])
	temp_rel = sensor_gyro_2['temperature'] - gyro_2_params['TC_G2_TREF']
	temp_rel_resample = np.linspace(gyro_2_params['TC_G2_TMIN']-gyro_2_params['TC_G2_TREF'], gyro_2_params['TC_G2_TMAX']-gyro_2_params['TC_G2_TREF'], 100)
	temp_resample = temp_rel_resample + gyro_2_params['TC_G2_TREF']

	# fit X axis
	coef_gyro_2_x = np.polyfit(temp_rel,sensor_gyro_2['x'],3)
	gyro_2_params['TC_G2_X3_0'] = coef_gyro_2_x[0]
	gyro_2_params['TC_G2_X2_0'] = coef_gyro_2_x[1]
	gyro_2_params['TC_G2_X1_0'] = coef_gyro_2_x[2]
	gyro_2_params['TC_G2_X0_0'] = coef_gyro_2_x[3]
	fit_coef_gyro_2_x = np.poly1d(coef_gyro_2_x)
	gyro_2_x_resample = fit_coef_gyro_2_x(temp_rel_resample)

	# fit Y axis
	coef_gyro_2_y = np.polyfit(temp_rel,sensor_gyro_2['y'],3)
	gyro_2_params['TC_G2_X3_1'] = coef_gyro_2_y[0]
	gyro_2_params['TC_G2_X2_1'] = coef_gyro_2_y[1]
	gyro_2_params['TC_G2_X1_1'] = coef_gyro_2_y[2]
	gyro_2_params['TC_G2_X0_1'] = coef_gyro_2_y[3]
	fit_coef_gyro_2_y = np.poly1d(coef_gyro_2_y)
	gyro_2_y_resample = fit_coef_gyro_2_y(temp_rel_resample)

	# fit Z axis
	coef_gyro_2_z = np.polyfit(temp_rel,sensor_gyro_2['z'],3)
	gyro_2_params['TC_G2_X3_2'] = coef_gyro_2_z[0]
	gyro_2_params['TC_G2_X2_2'] = coef_gyro_2_z[1]
	gyro_2_params['TC_G2_X1_2'] = coef_gyro_2_z[2]
	gyro_2_params['TC_G2_X0_2'] = coef_gyro_2_z[3]
	fit_coef_gyro_2_z = np.poly1d(coef_gyro_2_z)
	gyro_2_z_resample = fit_coef_gyro_2_z(temp_rel_resample)

	# gyro2 vs temperature
	plt.figure(3,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['x'],'b')
	plt.plot(temp_resample,gyro_2_x_resample,'r')
	plt.title('Gyro 2 Bias vs Temperature')
	plt.ylabel('X bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['y'],'b')
	plt.plot(temp_resample,gyro_2_y_resample,'r')
	plt.ylabel('Y bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_gyro_2['temperature'],sensor_gyro_2['z'],'b')
	plt.plot(temp_resample,gyro_2_z_resample,'r')
	plt.ylabel('Z bias (rad/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 0 thermal correction  parameters
accel_0_params = {
'TC_A0_ID':0,
'TC_A0_TMIN':0.0,
'TC_A0_TMAX':0.0,
'TC_A0_TREF':0.0,
'TC_A0_X0_0':0.0,
'TC_A0_X1_0':0.0,
'TC_A0_X2_0':0.0,
'TC_A0_X3_0':0.0,
'TC_A0_X0_1':0.0,
'TC_A0_X1_1':0.0,
'TC_A0_X2_1':0.0,
'TC_A0_X3_1':0.0,
'TC_A0_X0_2':0.0,
'TC_A0_X1_2':0.0,
'TC_A0_X2_2':0.0,
'TC_A0_X3_2':0.0,
'TC_A0_SCL_0':1.0,
'TC_A0_SCL_1':1.0,
'TC_A0_SCL_2':1.0
}

# curve fit the data for accel 0 corrections
if num_accels >= 1:
	accel_0_params['TC_A0_ID'] = int(np.median(sensor_accel_0['device_id']))

	# find the min, max and reference temperature
	accel_0_params['TC_A0_TMIN'] = np.amin(sensor_accel_0['temperature'])
	accel_0_params['TC_A0_TMAX'] = np.amax(sensor_accel_0['temperature'])
	accel_0_params['TC_A0_TREF'] = 0.5 * (accel_0_params['TC_A0_TMIN'] + accel_0_params['TC_A0_TMAX'])
	temp_rel = sensor_accel_0['temperature'] - accel_0_params['TC_A0_TREF']
	temp_rel_resample = np.linspace(accel_0_params['TC_A0_TMIN']-accel_0_params['TC_A0_TREF'], accel_0_params['TC_A0_TMAX']-accel_0_params['TC_A0_TREF'], 100)
	temp_resample = temp_rel_resample + accel_0_params['TC_A0_TREF']

	# fit X axis
	correction_x = sensor_accel_0['x'] - np.median(sensor_accel_0['x'])
	coef_accel_0_x = np.polyfit(temp_rel,correction_x,3)
	accel_0_params['TC_A0_X3_0'] = coef_accel_0_x[0]
	accel_0_params['TC_A0_X2_0'] = coef_accel_0_x[1]
	accel_0_params['TC_A0_X1_0'] = coef_accel_0_x[2]
	accel_0_params['TC_A0_X0_0'] = coef_accel_0_x[3]
	fit_coef_accel_0_x = np.poly1d(coef_accel_0_x)
	correction_x_resample = fit_coef_accel_0_x(temp_rel_resample)

	# fit Y axis
	correction_y = sensor_accel_0['y']-np.median(sensor_accel_0['y'])
	coef_accel_0_y = np.polyfit(temp_rel,correction_y,3)
	accel_0_params['TC_A0_X3_1'] = coef_accel_0_y[0]
	accel_0_params['TC_A0_X2_1'] = coef_accel_0_y[1]
	accel_0_params['TC_A0_X1_1'] = coef_accel_0_y[2]
	accel_0_params['TC_A0_X0_1'] = coef_accel_0_y[3]
	fit_coef_accel_0_y = np.poly1d(coef_accel_0_y)
	correction_y_resample = fit_coef_accel_0_y(temp_rel_resample)

	# fit Z axis
	correction_z = sensor_accel_0['z']-np.median(sensor_accel_0['z'])
	coef_accel_0_z = np.polyfit(temp_rel,correction_z,3)
	accel_0_params['TC_A0_X3_2'] = coef_accel_0_z[0]
	accel_0_params['TC_A0_X2_2'] = coef_accel_0_z[1]
	accel_0_params['TC_A0_X1_2'] = coef_accel_0_z[2]
	accel_0_params['TC_A0_X0_2'] = coef_accel_0_z[3]
	fit_coef_accel_0_z = np.poly1d(coef_accel_0_z)
	correction_z_resample = fit_coef_accel_0_z(temp_rel_resample)

	# accel 0 vs temperature
	plt.figure(4,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_accel_0['temperature'],correction_x,'b')
	plt.plot(temp_resample,correction_x_resample,'r')
	plt.title('Accel 0 Bias vs Temperature')
	plt.ylabel('X bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_accel_0['temperature'],correction_y,'b')
	plt.plot(temp_resample,correction_y_resample,'r')
	plt.ylabel('Y bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_accel_0['temperature'],correction_z,'b')
	plt.plot(temp_resample,correction_z_resample,'r')
	plt.ylabel('Z bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 1 thermal correction  parameters
accel_1_params = {
'TC_A1_ID':0,
'TC_A1_TMIN':0.0,
'TC_A1_TMAX':0.0,
'TC_A1_TREF':0.0,
'TC_A1_X0_0':0.0,
'TC_A1_X1_0':0.0,
'TC_A1_X2_0':0.0,
'TC_A1_X3_0':0.0,
'TC_A1_X0_1':0.0,
'TC_A1_X1_1':0.0,
'TC_A1_X2_1':0.0,
'TC_A1_X3_1':0.0,
'TC_A1_X0_2':0.0,
'TC_A1_X1_2':0.0,
'TC_A1_X2_2':0.0,
'TC_A1_X3_2':0.0,
'TC_A1_SCL_0':1.0,
'TC_A1_SCL_1':1.0,
'TC_A1_SCL_2':1.0
}

# curve fit the data for accel 1 corrections
if num_accels >= 2:
	accel_1_params['TC_A1_ID'] = int(np.median(sensor_accel_1['device_id']))

	# find the min, max and reference temperature
	accel_1_params['TC_A1_TMIN'] = np.amin(sensor_accel_1['temperature'])
	accel_1_params['TC_A1_TMAX'] = np.amax(sensor_accel_1['temperature'])
	accel_1_params['TC_A1_TREF'] = 0.5 * (accel_1_params['TC_A1_TMIN'] + accel_1_params['TC_A1_TMAX'])
	temp_rel = sensor_accel_1['temperature'] - accel_1_params['TC_A1_TREF']
	temp_rel_resample = np.linspace(accel_1_params['TC_A1_TMIN']-accel_1_params['TC_A1_TREF'], accel_1_params['TC_A1_TMAX']-accel_1_params['TC_A1_TREF'], 100)
	temp_resample = temp_rel_resample + accel_1_params['TC_A1_TREF']

	# fit X axis
	correction_x = sensor_accel_1['x']-np.median(sensor_accel_1['x'])
	coef_accel_1_x = np.polyfit(temp_rel,correction_x,3)
	accel_1_params['TC_A1_X3_0'] = coef_accel_1_x[0]
	accel_1_params['TC_A1_X2_0'] = coef_accel_1_x[1]
	accel_1_params['TC_A1_X1_0'] = coef_accel_1_x[2]
	accel_1_params['TC_A1_X0_0'] = coef_accel_1_x[3]
	fit_coef_accel_1_x = np.poly1d(coef_accel_1_x)
	correction_x_resample = fit_coef_accel_1_x(temp_rel_resample)

	# fit Y axis
	correction_y = sensor_accel_1['y']-np.median(sensor_accel_1['y'])
	coef_accel_1_y = np.polyfit(temp_rel,correction_y,3)
	accel_1_params['TC_A1_X3_1'] = coef_accel_1_y[0]
	accel_1_params['TC_A1_X2_1'] = coef_accel_1_y[1]
	accel_1_params['TC_A1_X1_1'] = coef_accel_1_y[2]
	accel_1_params['TC_A1_X0_1'] = coef_accel_1_y[3]
	fit_coef_accel_1_y = np.poly1d(coef_accel_1_y)
	correction_y_resample = fit_coef_accel_1_y(temp_rel_resample)

	# fit Z axis
	correction_z = (sensor_accel_1['z'])-np.median(sensor_accel_1['z'])
	coef_accel_1_z = np.polyfit(temp_rel,correction_z,3)
	accel_1_params['TC_A1_X3_2'] = coef_accel_1_z[0]
	accel_1_params['TC_A1_X2_2'] = coef_accel_1_z[1]
	accel_1_params['TC_A1_X1_2'] = coef_accel_1_z[2]
	accel_1_params['TC_A1_X0_2'] = coef_accel_1_z[3]
	fit_coef_accel_1_z = np.poly1d(coef_accel_1_z)
	correction_z_resample = fit_coef_accel_1_z(temp_rel_resample)

	# accel 1 vs temperature
	plt.figure(5,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_accel_1['temperature'],correction_x,'b')
	plt.plot(temp_resample,correction_x_resample,'r')
	plt.title('Accel 1 Bias vs Temperature')
	plt.ylabel('X bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_accel_1['temperature'],correction_y,'b')
	plt.plot(temp_resample,correction_y_resample,'r')
	plt.ylabel('Y bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_accel_1['temperature'],correction_z,'b')
	plt.plot(temp_resample,correction_z_resample,'r')
	plt.ylabel('Z bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of accel 2 thermal correction  parameters
accel_2_params = {
'TC_A2_ID':0,
'TC_A2_TMIN':0.0,
'TC_A2_TMAX':0.0,
'TC_A2_TREF':0.0,
'TC_A2_X0_0':0.0,
'TC_A2_X1_0':0.0,
'TC_A2_X2_0':0.0,
'TC_A2_X3_0':0.0,
'TC_A2_X0_1':0.0,
'TC_A2_X1_1':0.0,
'TC_A2_X2_1':0.0,
'TC_A2_X3_1':0.0,
'TC_A2_X0_2':0.0,
'TC_A2_X1_2':0.0,
'TC_A2_X2_2':0.0,
'TC_A2_X3_2':0.0,
'TC_A2_SCL_0':1.0,
'TC_A2_SCL_1':1.0,
'TC_A2_SCL_2':1.0
}

# curve fit the data for accel 2 corrections
if num_accels >= 3:
	accel_2_params['TC_A2_ID'] = int(np.median(sensor_accel_2['device_id']))

	# find the min, max and reference temperature
	accel_2_params['TC_A2_TMIN'] = np.amin(sensor_accel_2['temperature'])
	accel_2_params['TC_A2_TMAX'] = np.amax(sensor_accel_2['temperature'])
	accel_2_params['TC_A2_TREF'] = 0.5 * (accel_2_params['TC_A2_TMIN'] + accel_2_params['TC_A2_TMAX'])
	temp_rel = sensor_accel_2['temperature'] - accel_2_params['TC_A2_TREF']
	temp_rel_resample = np.linspace(accel_2_params['TC_A2_TMIN']-accel_2_params['TC_A2_TREF'], accel_2_params['TC_A2_TMAX']-accel_2_params['TC_A2_TREF'], 100)
	temp_resample = temp_rel_resample + accel_2_params['TC_A2_TREF']

	# fit X axis
	correction_x = sensor_accel_2['x']-np.median(sensor_accel_2['x'])
	coef_accel_2_x = np.polyfit(temp_rel,correction_x,3)
	accel_2_params['TC_A2_X3_0'] = coef_accel_2_x[0]
	accel_2_params['TC_A2_X2_0'] = coef_accel_2_x[1]
	accel_2_params['TC_A2_X1_0'] = coef_accel_2_x[2]
	accel_2_params['TC_A2_X0_0'] = coef_accel_2_x[3]
	fit_coef_accel_2_x = np.poly1d(coef_accel_2_x)
	correction_x_resample = fit_coef_accel_2_x(temp_rel_resample)

	# fit Y axis
	correction_y = sensor_accel_2['y']-np.median(sensor_accel_2['y'])
	coef_accel_2_y = np.polyfit(temp_rel,correction_y,3)
	accel_2_params['TC_A2_X3_1'] = coef_accel_2_y[0]
	accel_2_params['TC_A2_X2_1'] = coef_accel_2_y[1]
	accel_2_params['TC_A2_X1_1'] = coef_accel_2_y[2]
	accel_2_params['TC_A2_X0_1'] = coef_accel_2_y[3]
	fit_coef_accel_2_y = np.poly1d(coef_accel_2_y)
	correction_y_resample = fit_coef_accel_2_y(temp_rel_resample)

	# fit Z axis
	correction_z = sensor_accel_2['z']-np.median(sensor_accel_2['z'])
	coef_accel_2_z = np.polyfit(temp_rel,correction_z,3)
	accel_2_params['TC_A2_X3_2'] = coef_accel_2_z[0]
	accel_2_params['TC_A2_X2_2'] = coef_accel_2_z[1]
	accel_2_params['TC_A2_X1_2'] = coef_accel_2_z[2]
	accel_2_params['TC_A2_X0_2'] = coef_accel_2_z[3]
	fit_coef_accel_2_z = np.poly1d(coef_accel_2_z)
	correction_z_resample = fit_coef_accel_2_z(temp_rel_resample)

	# accel 2 vs temperature
	plt.figure(6,figsize=(20,13))

	# draw plots
	plt.subplot(3,1,1)
	plt.plot(sensor_accel_2['temperature'],correction_x,'b')
	plt.plot(temp_resample,correction_x_resample,'r')
	plt.title('Accel 2 Bias vs Temperature')
	plt.ylabel('X bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,2)
	plt.plot(sensor_accel_2['temperature'],correction_y,'b')
	plt.plot(temp_resample,correction_y_resample,'r')
	plt.ylabel('Y bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	# draw plots
	plt.subplot(3,1,3)
	plt.plot(sensor_accel_2['temperature'],correction_z,'b')
	plt.plot(temp_resample,correction_z_resample,'r')
	plt.ylabel('Z bias (m/s/s)')
	plt.xlabel('temperature (degC)')
	plt.grid()

	pp.savefig()

#################################################################################

#################################################################################

# define data dictionary of baro 0 thermal correction  parameters
baro_0_params = {
'TC_B0_ID':0,
'TC_B0_TMIN':0.0,
'TC_B0_TMAX':0.0,
'TC_B0_TREF':0.0,
'TC_B0_X0':0.0,
'TC_B0_X1':0.0,
'TC_B0_X2':0.0,
'TC_B0_X3':0.0,
'TC_B0_X4':0.0,
'TC_B0_X5':0.0,
'TC_B0_SCL':1.0,
}

# curve fit the data for baro 0 corrections
baro_0_params['TC_B0_ID'] = int(np.median(sensor_baro_0['device_id']))

# find the min, max and reference temperature
baro_0_params['TC_B0_TMIN'] = np.amin(sensor_baro_0['temperature'])
baro_0_params['TC_B0_TMAX'] = np.amax(sensor_baro_0['temperature'])
baro_0_params['TC_B0_TREF'] = 0.5 * (baro_0_params['TC_B0_TMIN'] + baro_0_params['TC_B0_TMAX'])
temp_rel = sensor_baro_0['temperature'] - baro_0_params['TC_B0_TREF']
temp_rel_resample = np.linspace(baro_0_params['TC_B0_TMIN']-baro_0_params['TC_B0_TREF'], baro_0_params['TC_B0_TMAX']-baro_0_params['TC_B0_TREF'], 100)
temp_resample = temp_rel_resample + baro_0_params['TC_B0_TREF']

# fit data
median_pressure = np.median(sensor_baro_0['pressure']);
coef_baro_0_x = np.polyfit(temp_rel,100*(sensor_baro_0['pressure']-median_pressure),5) # convert from hPa to Pa
baro_0_params['TC_B0_X5'] = coef_baro_0_x[0]
baro_0_params['TC_B0_X4'] = coef_baro_0_x[1]
baro_0_params['TC_B0_X3'] = coef_baro_0_x[2]
baro_0_params['TC_B0_X2'] = coef_baro_0_x[3]
baro_0_params['TC_B0_X1'] = coef_baro_0_x[4]
baro_0_params['TC_B0_X0'] = coef_baro_0_x[5]
fit_coef_baro_0_x = np.poly1d(coef_baro_0_x)
baro_0_x_resample = fit_coef_baro_0_x(temp_rel_resample)

# baro 0 vs temperature
plt.figure(7,figsize=(20,13))

# draw plots
plt.plot(sensor_baro_0['temperature'],100*sensor_baro_0['pressure']-100*median_pressure,'b')
plt.plot(temp_resample,baro_0_x_resample,'r')
plt.title('Baro 0 Bias vs Temperature')
plt.ylabel('X bias (Pa)')
plt.xlabel('temperature (degC)')
plt.grid()

pp.savefig()

#################################################################################

# close the pdf file
pp.close()

# clase all figures
plt.close("all")

# write correction parameters to file
test_results_filename = ulog_file_name + ".params"
file = open(test_results_filename,"w")
file.write("# Sensor thermal compensation parameters\n")
file.write("#\n")
file.write("# Vehicle-Id Component-Id Name Value Type\n")

# accel 0 corrections
key_list_accel = list(accel_0_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_0_params[key])+"\t"+type+"\n")

# accel 1 corrections
key_list_accel = list(accel_1_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A1_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_1_params[key])+"\t"+type+"\n")

# accel 2 corrections
key_list_accel = list(accel_2_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_A2_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(accel_2_params[key])+"\t"+type+"\n")

# baro 0 corrections
key_list_accel = list(baro_0_params.keys())
key_list_accel.sort
for key in key_list_accel:
    if key == 'TC_B0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(baro_0_params[key])+"\t"+type+"\n")

# gyro 0 corrections
key_list_gyro = list(gyro_0_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G0_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_0_params[key])+"\t"+type+"\n")

# gyro 1 corrections
key_list_gyro = list(gyro_1_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G1_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_1_params[key])+"\t"+type+"\n")

# gyro 2 corrections
key_list_gyro = list(gyro_2_params.keys())
key_list_gyro.sort()
for key in key_list_gyro:
    if key == 'TC_G2_ID':
        type = "6"
    else:
        type = "9"
    file.write("1"+"\t"+"1"+"\t"+key+"\t"+str(gyro_2_params[key])+"\t"+type+"\n")

file.close()

print('Correction parameters written to ' + test_results_filename)
print('Plots saved to ' + output_plot_filename)
