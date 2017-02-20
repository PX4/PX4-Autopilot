#! /usr/bin/env python

from __future__ import print_function

import argparse
import os
import matplotlib.pyplot as plt
import numpy as np

from pyulog import *

"""
Performs a health assessment on the ecl EKF navigation estimator data contained in a an ULog file
Outputs a health assessment summary in a csv file named <inputfilename>.mdat.csv
Outputs summary plots in a pdf file named <inputfilename>.pdf
"""

parser = argparse.ArgumentParser(description='Analyse the estimator_status and ekf2_innovation message data')
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

# extract data from innovations and status messages
for d in data:
    if d.name == 'estimator_status':
        estimator_status = d.data
        print('found estimator_status data')
for d in data:
    if d.name == 'ekf2_innovations':
        ekf2_innovations = d.data
        print('found ekf2_innovation data')

# extract data from sensor reflight check message
sensor_preflight = {}
for d in data:
    if d.name == 'sensor_preflight':
        sensor_preflight = d.data
        print('found sensor_preflight data')

# create summary plots
# save the plots to PDF
from matplotlib.backends.backend_pdf import PdfPages
output_plot_filename = ulog_file_name + ".pdf"
pp = PdfPages(output_plot_filename)

# plot IMU consistency data
if ('accel_inconsistency_m_s_s' in sensor_preflight.keys()) and ('gyro_inconsistency_rad_s' in sensor_preflight.keys()):
    plt.figure(0,figsize=(20,13))
    plt.subplot(2,1,1)
    plt.plot(sensor_preflight['accel_inconsistency_m_s_s'],'b')
    plt.title('IMU Consistency Check Levels')
    plt.ylabel('acceleration (m/s/s)')
    plt.xlabel('data index')
    plt.grid()
    plt.subplot(2,1,2)
    plt.plot(sensor_preflight['gyro_inconsistency_rad_s'],'b')
    plt.ylabel('angular rate (rad/s)')
    plt.xlabel('data index')
    pp.savefig()

# vertical velocity and position innovations
plt.figure(1,figsize=(20,13))

# generate max, min and 1-std metadata
innov_time = 1e-6*ekf2_innovations['timestamp']
status_time = 1e-6*estimator_status['timestamp']

# generate metadata for velocity innovations
innov_2_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[2]'])
innov_2_max_time = innov_time[innov_2_max_arg]
innov_2_max = np.amax(ekf2_innovations['vel_pos_innov[2]'])

innov_2_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[2]'])
innov_2_min_time = innov_time[innov_2_min_arg]
innov_2_min = np.amin(ekf2_innovations['vel_pos_innov[2]'])

s_innov_2_max = str(round(innov_2_max,2))
s_innov_2_min = str(round(innov_2_min,2))
#s_innov_2_std = str(round(np.std(ekf2_innovations['vel_pos_innov[2]']),2))

# generate metadata for position innovations
innov_5_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[5]'])
innov_5_max_time = innov_time[innov_5_max_arg]
innov_5_max = np.amax(ekf2_innovations['vel_pos_innov[5]'])

innov_5_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[5]'])
innov_5_min_time = innov_time[innov_5_min_arg]
innov_5_min = np.amin(ekf2_innovations['vel_pos_innov[5]'])

s_innov_5_max = str(round(innov_5_max,2))
s_innov_5_min = str(round(innov_5_min,2))
#s_innov_5_std = str(round(np.std(ekf2_innovations['vel_pos_innov[5]']),2))

# generate plot for vertical velocity innovations
plt.subplot(2,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[2]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[2]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[2]']),'r')
plt.title('Vertical Innovations')
plt.ylabel('Down Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_2_max_time, innov_2_max, 'max='+s_innov_2_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_2_min_time, innov_2_min, 'min='+s_innov_2_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_2_std],loc='upper left',frameon=False)

# generate plot for vertical position innovations
plt.subplot(2,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[5]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[5]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[5]']),'r')
plt.ylabel('Down Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_5_max_time, innov_5_max, 'max='+s_innov_5_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_5_min_time, innov_5_min, 'min='+s_innov_5_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_5_std],loc='upper left',frameon=False)

pp.savefig()

# horizontal velocity innovations
plt.figure(2,figsize=(20,13))

# generate North axis metadata
innov_0_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[0]'])
innov_0_max_time = innov_time[innov_0_max_arg]
innov_0_max = np.amax(ekf2_innovations['vel_pos_innov[0]'])

innov_0_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[0]'])
innov_0_min_time = innov_time[innov_0_min_arg]
innov_0_min = np.amin(ekf2_innovations['vel_pos_innov[0]'])

s_innov_0_max = str(round(innov_0_max,2))
s_innov_0_min = str(round(innov_0_min,2))
#s_innov_0_std = str(round(np.std(ekf2_innovations['vel_pos_innov[0]']),2))

# Generate East axis metadata
innov_1_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[1]'])
innov_1_max_time = innov_time[innov_1_max_arg]
innov_1_max = np.amax(ekf2_innovations['vel_pos_innov[1]'])

innov_1_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[1]'])
innov_1_min_time = innov_time[innov_1_min_arg]
innov_1_min = np.amin(ekf2_innovations['vel_pos_innov[1]'])

s_innov_1_max = str(round(innov_1_max,2))
s_innov_1_min = str(round(innov_1_min,2))
#s_innov_1_std = str(round(np.std(ekf2_innovations['vel_pos_innov[1]']),2))

# draw plots
plt.subplot(2,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[0]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[0]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[0]']),'r')
plt.title('Horizontal Velocity  Innovations')
plt.ylabel('North Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_0_max_time, innov_0_max, 'max='+s_innov_0_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_0_min_time, innov_0_min, 'min='+s_innov_0_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_0_std],loc='upper left',frameon=False)

plt.subplot(2,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[1]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[1]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[1]']),'r')
plt.ylabel('East Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_1_max_time, innov_1_max, 'max='+s_innov_1_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_1_min_time, innov_1_min, 'min='+s_innov_1_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_1_std],loc='upper left',frameon=False)

pp.savefig()

# horizontal position innovations
plt.figure(3,figsize=(20,13))

# generate North axis metadata
innov_3_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[3]'])
innov_3_max_time = innov_time[innov_3_max_arg]
innov_3_max = np.amax(ekf2_innovations['vel_pos_innov[3]'])

innov_3_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[3]'])
innov_3_min_time = innov_time[innov_3_min_arg]
innov_3_min = np.amin(ekf2_innovations['vel_pos_innov[3]'])

s_innov_3_max = str(round(innov_3_max,2))
s_innov_3_min = str(round(innov_3_min,2))
#s_innov_3_std = str(round(np.std(ekf2_innovations['vel_pos_innov[3]']),2))

# generate East axis metadata
innov_4_max_arg = np.argmax(ekf2_innovations['vel_pos_innov[4]'])
innov_4_max_time = innov_time[innov_4_max_arg]
innov_4_max = np.amax(ekf2_innovations['vel_pos_innov[4]'])

innov_4_min_arg = np.argmin(ekf2_innovations['vel_pos_innov[4]'])
innov_4_min_time = innov_time[innov_4_min_arg]
innov_4_min = np.amin(ekf2_innovations['vel_pos_innov[4]'])

s_innov_4_max = str(round(innov_4_max,2))
s_innov_4_min = str(round(innov_4_min,2))
#s_innov_4_std = str(round(np.std(ekf2_innovations['vel_pos_innov[4]']),2))

# generate plots

plt.subplot(2,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[3]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[3]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[3]']),'r')
plt.title('Horizontal Position Innovations')
plt.ylabel('North Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_3_max_time, innov_3_max, 'max='+s_innov_3_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_3_min_time, innov_3_min, 'min='+s_innov_3_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_3_std],loc='upper left',frameon=False)

plt.subplot(2,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['vel_pos_innov[4]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['vel_pos_innov_var[4]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['vel_pos_innov_var[4]']),'r')
plt.ylabel('East Pos (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_4_max_time, innov_4_max, 'max='+s_innov_4_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_4_min_time, innov_4_min, 'min='+s_innov_4_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_4_std],loc='upper left',frameon=False)

pp.savefig()

# manetometer innovations
plt.figure(4,figsize=(20,13))

# generate X axis metadata
innov_0_max_arg = np.argmax(ekf2_innovations['mag_innov[0]'])
innov_0_max_time = innov_time[innov_0_max_arg]
innov_0_max = np.amax(ekf2_innovations['mag_innov[0]'])

innov_0_min_arg = np.argmin(ekf2_innovations['mag_innov[0]'])
innov_0_min_time = innov_time[innov_0_min_arg]
innov_0_min = np.amin(ekf2_innovations['mag_innov[0]'])

s_innov_0_max = str(round(innov_0_max,3))
s_innov_0_min = str(round(innov_0_min,3))
#s_innov_0_std = str(round(np.std(ekf2_innovations['mag_innov[0]']),3))

# generate Y axis metadata
innov_1_max_arg = np.argmax(ekf2_innovations['mag_innov[1]'])
innov_1_max_time = innov_time[innov_1_max_arg]
innov_1_max = np.amax(ekf2_innovations['mag_innov[1]'])

innov_1_min_arg = np.argmin(ekf2_innovations['mag_innov[1]'])
innov_1_min_time = innov_time[innov_1_min_arg]
innov_1_min = np.amin(ekf2_innovations['mag_innov[1]'])

s_innov_1_max = str(round(innov_1_max,3))
s_innov_1_min = str(round(innov_1_min,3))
#s_innov_1_std = str(round(np.std(ekf2_innovations['mag_innov[1]']),3))

# generate Z axis metadata
innov_2_max_arg = np.argmax(ekf2_innovations['mag_innov[2]'])
innov_2_max_time = innov_time[innov_2_max_arg]
innov_2_max = np.amax(ekf2_innovations['mag_innov[2]'])

innov_2_min_arg = np.argmin(ekf2_innovations['mag_innov[2]'])
innov_2_min_time = innov_time[innov_2_min_arg]
innov_2_min = np.amin(ekf2_innovations['mag_innov[2]'])

s_innov_2_max = str(round(innov_2_max,3))
s_innov_2_min = str(round(innov_2_min,3))
#s_innov_2_std = str(round(np.std(ekf2_innovations['mag_innov[0]']),3))

# draw plots
plt.subplot(3,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['mag_innov[0]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['mag_innov_var[0]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['mag_innov_var[0]']),'r')
plt.title('Magnetometer Innovations')
plt.ylabel('X (Gauss)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_0_max_time, innov_0_max, 'max='+s_innov_0_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_0_min_time, innov_0_min, 'min='+s_innov_0_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_0_std],loc='upper left',frameon=False)

plt.subplot(3,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['mag_innov[1]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['mag_innov_var[1]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['mag_innov_var[1]']),'r')
plt.ylabel('Y (Gauss)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_1_max_time, innov_1_max, 'max='+s_innov_1_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_1_min_time, innov_1_min, 'min='+s_innov_1_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_1_std],loc='upper left',frameon=False)

plt.subplot(3,1,3)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['mag_innov[2]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['mag_innov_var[2]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['mag_innov_var[2]']),'r')
plt.ylabel('Z (Gauss)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_2_max_time, innov_2_max, 'max='+s_innov_2_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_2_min_time, innov_2_min, 'min='+s_innov_2_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_2_std],loc='upper left',frameon=False)

pp.savefig()

# magnetic heading innovations
plt.figure(5,figsize=(20,13))

# generate metadata
innov_0_max_arg = np.argmax(ekf2_innovations['heading_innov'])
innov_0_max_time = innov_time[innov_0_max_arg]
innov_0_max = np.amax(ekf2_innovations['heading_innov'])

innov_0_min_arg = np.argmin(ekf2_innovations['heading_innov'])
innov_0_min_time = innov_time[innov_0_min_arg]
innov_0_min = np.amin(ekf2_innovations['heading_innov'])

s_innov_0_max = str(round(innov_0_max,3))
s_innov_0_min = str(round(innov_0_min,3))
#s_innov_0_std = str(round(np.std(ekf2_innovations['heading_innov']),3))

# draw plot
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['heading_innov'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['heading_innov_var']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['heading_innov_var']),'r')
plt.title('Magnetic Heading Innovations')
plt.ylabel('Heaing (rad)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(innov_0_max_time, innov_0_max, 'max='+s_innov_0_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(innov_0_min_time, innov_0_min, 'min='+s_innov_0_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_innov_0_std],loc='upper left',frameon=False)

pp.savefig()

# air data innovations
plt.figure(6,figsize=(20,13))

# generate airspeed metadata
airspeed_innov_max_arg = np.argmax(ekf2_innovations['airspeed_innov'])
airspeed_innov_max_time = innov_time[airspeed_innov_max_arg]
airspeed_innov_max = np.amax(ekf2_innovations['airspeed_innov'])

airspeed_innov_min_arg = np.argmin(ekf2_innovations['airspeed_innov'])
airspeed_innov_min_time = innov_time[airspeed_innov_min_arg]
airspeed_innov_min = np.amin(ekf2_innovations['airspeed_innov'])

s_airspeed_innov_max = str(round(airspeed_innov_max,3))
s_airspeed_innov_min = str(round(airspeed_innov_min,3))

# generate sideslip metadata
beta_innov_max_arg = np.argmax(ekf2_innovations['beta_innov'])
beta_innov_max_time = innov_time[beta_innov_max_arg]
beta_innov_max = np.amax(ekf2_innovations['beta_innov'])

beta_innov_min_arg = np.argmin(ekf2_innovations['beta_innov'])
beta_innov_min_time = innov_time[beta_innov_min_arg]
beta_innov_min = np.amin(ekf2_innovations['beta_innov'])

s_beta_innov_max = str(round(beta_innov_max,3))
s_beta_innov_min = str(round(beta_innov_min,3))

# draw plots
plt.subplot(2,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['airspeed_innov'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['airspeed_innov_var']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['airspeed_innov_var']),'r')
plt.title('True Airspeed Innovations')
plt.ylabel('innovation (m/sec)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(airspeed_innov_max_time, airspeed_innov_max, 'max='+s_airspeed_innov_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(airspeed_innov_min_time, airspeed_innov_min, 'min='+s_airspeed_innov_min, fontsize=12, horizontalalignment='left', verticalalignment='top')

plt.subplot(2,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['beta_innov'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['beta_innov_var']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['beta_innov_var']),'r')
plt.title('Sythetic Sideslip Innovations')
plt.ylabel('innovation (rad)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(beta_innov_max_time, beta_innov_max, 'max='+s_beta_innov_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(beta_innov_min_time, beta_innov_min, 'min='+s_beta_innov_min, fontsize=12, horizontalalignment='left', verticalalignment='top')

pp.savefig()

# optical flow innovations
plt.figure(7,figsize=(20,13))

# generate X axis metadata
flow_innov_x_max_arg = np.argmax(ekf2_innovations['flow_innov[0]'])
flow_innov_x_max_time = innov_time[flow_innov_x_max_arg]
flow_innov_x_max = np.amax(ekf2_innovations['flow_innov[0]'])

flow_innov_x_min_arg = np.argmin(ekf2_innovations['flow_innov[0]'])
flow_innov_x_min_time = innov_time[flow_innov_x_min_arg]
flow_innov_x_min = np.amin(ekf2_innovations['flow_innov[0]'])

s_flow_innov_x_max = str(round(flow_innov_x_max,3))
s_flow_innov_x_min = str(round(flow_innov_x_min,3))
#s_flow_innov_x_std = str(round(np.std(ekf2_innovations['flow_innov[0]']),3))

# generate Y axis metadata
flow_innov_y_max_arg = np.argmax(ekf2_innovations['flow_innov[1]'])
flow_innov_y_max_time = innov_time[flow_innov_y_max_arg]
flow_innov_y_max = np.amax(ekf2_innovations['flow_innov[1]'])

flow_innov_y_min_arg = np.argmin(ekf2_innovations['flow_innov[1]'])
flow_innov_y_min_time = innov_time[flow_innov_y_min_arg]
flow_innov_y_min = np.amin(ekf2_innovations['flow_innov[1]'])

s_flow_innov_y_max = str(round(flow_innov_y_max,3))
s_flow_innov_y_min = str(round(flow_innov_y_min,3))
#s_flow_innov_y_std = str(round(np.std(ekf2_innovations['flow_innov[1]']),3))

# draw plots
plt.subplot(2,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['flow_innov[0]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['flow_innov_var[0]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['flow_innov_var[0]']),'r')
plt.title('Optical Flow Innovations')
plt.ylabel('X (rad/sec)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(flow_innov_x_max_time, flow_innov_x_max, 'max='+s_flow_innov_x_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(flow_innov_x_min_time, flow_innov_x_min, 'min='+s_flow_innov_x_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_flow_innov_x_std],loc='upper left',frameon=False)

plt.subplot(2,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'],ekf2_innovations['flow_innov[1]'],'b')
plt.plot(1e-6*ekf2_innovations['timestamp'],np.sqrt(ekf2_innovations['flow_innov_var[1]']),'r')
plt.plot(1e-6*ekf2_innovations['timestamp'],-np.sqrt(ekf2_innovations['flow_innov_var[1]']),'r')
plt.ylabel('Y (rad/sec)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(flow_innov_y_max_time, flow_innov_y_max, 'max='+s_flow_innov_y_max, fontsize=12, horizontalalignment='left', verticalalignment='bottom')
plt.text(flow_innov_y_min_time, flow_innov_y_min, 'min='+s_flow_innov_y_min, fontsize=12, horizontalalignment='left', verticalalignment='top')
#plt.legend(['std='+s_flow_innov_y_std],loc='upper left',frameon=False)

pp.savefig()

# generate metadata for the normalised innovation consistency test levels
# a value > 1.0 means the measurement data for that test has been rejected by the EKF

# magnetometer data
mag_test_max_arg = np.argmax(estimator_status['mag_test_ratio'])
mag_test_max_time = status_time[mag_test_max_arg]
mag_test_max = np.amax(estimator_status['mag_test_ratio'])
mag_test_mean = np.mean(estimator_status['mag_test_ratio'])

# velocity data (GPS)
vel_test_max_arg = np.argmax(estimator_status['vel_test_ratio'])
vel_test_max_time = status_time[vel_test_max_arg]
vel_test_max = np.amax(estimator_status['vel_test_ratio'])
vel_test_mean = np.mean(estimator_status['vel_test_ratio'])

# horizontal position data (GPS or external vision)
pos_test_max_arg = np.argmax(estimator_status['pos_test_ratio'])
pos_test_max_time = status_time[pos_test_max_arg]
pos_test_max = np.amax(estimator_status['pos_test_ratio'])
pos_test_mean = np.mean(estimator_status['pos_test_ratio'])

# height data (Barometer, GPS or rangefinder)
hgt_test_max_arg = np.argmax(estimator_status['hgt_test_ratio'])
hgt_test_max_time = status_time[hgt_test_max_arg]
hgt_test_max = np.amax(estimator_status['hgt_test_ratio'])
hgt_test_mean = np.mean(estimator_status['hgt_test_ratio'])

# airspeed data
tas_test_max_arg = np.argmax(estimator_status['tas_test_ratio'])
tas_test_max_time = status_time[tas_test_max_arg]
tas_test_max = np.amax(estimator_status['tas_test_ratio'])
tas_test_mean = np.mean(estimator_status['tas_test_ratio'])

# height above ground data (rangefinder)
hagl_test_max_arg = np.argmax(estimator_status['hagl_test_ratio'])
hagl_test_max_time = status_time[hagl_test_max_arg]
hagl_test_max = np.amax(estimator_status['hagl_test_ratio'])
hagl_test_mean = np.mean(estimator_status['hagl_test_ratio'])

# plot normalised innovation test levels
plt.figure(8,figsize=(20,13))

if tas_test_max == 0.0:
    n_plots = 3
else:
    n_plots = 4

plt.subplot(n_plots,1,1)
plt.plot(status_time,estimator_status['mag_test_ratio'],'b')
plt.title('Normalised Innovation Test Levels')
plt.ylabel('mag')
plt.xlabel('time (sec)')
plt.grid()
plt.text(mag_test_max_time, mag_test_max, 'max='+str(round(mag_test_max,2))+' , mean='+str(round(mag_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='b')

plt.subplot(n_plots,1,2)
plt.plot(status_time,estimator_status['vel_test_ratio'],'b')
plt.plot(status_time,estimator_status['pos_test_ratio'],'r')
plt.ylabel('vel,pos')
plt.xlabel('time (sec)')
plt.grid()
plt.text(vel_test_max_time, vel_test_max, 'vel max='+str(round(vel_test_max,2))+' , mean='+str(round(vel_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='b')
plt.text(pos_test_max_time, pos_test_max, 'pos max='+str(round(pos_test_max,2))+' , mean='+str(round(pos_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='r')

plt.subplot(n_plots,1,3)
plt.plot(status_time,estimator_status['hgt_test_ratio'],'b')
plt.ylabel('hgt')
plt.xlabel('time (sec)')
plt.grid()
plt.text(hgt_test_max_time, hgt_test_max, 'hgt max='+str(round(hgt_test_max,2))+' , mean='+str(round(hgt_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='b')

if hagl_test_max > 0.0:
    plt.plot(status_time,estimator_status['hagl_test_ratio'],'r')
    plt.text(hagl_test_max_time, hagl_test_max, 'hagl max='+str(round(hagl_test_max,2))+' , mean='+str(round(hagl_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='r')
    plt.ylabel('hgt,HAGL')

if n_plots == 4:
    plt.subplot(n_plots,1,4)
    plt.plot(status_time,estimator_status['tas_test_ratio'],'b')
    plt.ylabel('TAS')
    plt.xlabel('time (sec)')
    plt.grid()
    plt.text(tas_test_max_time, tas_test_max, 'max='+str(round(tas_test_max,2))+' , mean='+str(round(tas_test_mean,2)), fontsize=12, horizontalalignment='left', verticalalignment='bottom',color='b')

pp.savefig()

# extract control mode metadata from estimator_status.control_mode_flags
# 0 - true if the filter tilt alignment is complete
# 1 - true if the filter yaw alignment is complete
# 2 - true if GPS measurements are being fused
# 3 - true if optical flow measurements are being fused
# 4 - true if a simple magnetic yaw heading is being fused
# 5 - true if 3-axis magnetometer measurement are being fused
# 6 - true if synthetic magnetic declination measurements are being fused
# 7 - true when the vehicle is airborne
# 8 - true when wind velocity is being estimated
# 9 - true when baro height is being fused as a primary height reference
# 10 - true when range finder height is being fused as a primary height reference
# 11 - true when range finder height is being fused as a primary height reference
# 12 - true when local position data from external vision is being fused
# 13 - true when yaw data from external vision measurements is being fused
# 14 - true when height data from external vision measurements is being fused
tilt_aligned    = ((2**0 & estimator_status['control_mode_flags']) > 0)*1
yaw_aligned     = ((2**1 & estimator_status['control_mode_flags']) > 0)*1
using_gps       = ((2**2 & estimator_status['control_mode_flags']) > 0)*1
using_optflow   = ((2**3 & estimator_status['control_mode_flags']) > 0)*1
using_magyaw    = ((2**4 & estimator_status['control_mode_flags']) > 0)*1
using_mag3d     = ((2**5 & estimator_status['control_mode_flags']) > 0)*1
using_magdecl   = ((2**6 & estimator_status['control_mode_flags']) > 0)*1
airborne        = ((2**7 & estimator_status['control_mode_flags']) > 0)*1
estimating_wind = ((2**8 & estimator_status['control_mode_flags']) > 0)*1
using_barohgt   = ((2**9 & estimator_status['control_mode_flags']) > 0)*1
using_rnghgt    = ((2**10 & estimator_status['control_mode_flags']) > 0)*1
using_gpshgt    = ((2**11 & estimator_status['control_mode_flags']) > 0)*1
using_evpos     = ((2**12 & estimator_status['control_mode_flags']) > 0)*1
using_evyaw     = ((2**13 & estimator_status['control_mode_flags']) > 0)*1
using_evhgt     = ((2**14 & estimator_status['control_mode_flags']) > 0)*1

# calculate in-air transition time
if (np.amin(airborne) < 0.5) and (np.amax(airborne) > 0.5):
    in_air_transtion_time_arg = np.argmax(np.diff(airborne))
    in_air_transition_time = status_time[in_air_transtion_time_arg]
elif (np.amax(airborne) > 0.5):
    in_air_transition_time = np.amin(status_time)
    print('log starts while in-air at '+str(round(in_air_transition_time,1))+' sec')
else:
    in_air_transition_time = float('NaN')
    print('always on ground')

# calculate on-ground transition time
if (np.amin(np.diff(airborne)) < 0.0):
    on_ground_transition_time_arg = np.argmin(np.diff(airborne))
    on_ground_transition_time = status_time[on_ground_transition_time_arg]
elif (np.amax(airborne) > 0.5):
    on_ground_transition_time = np.amax(status_time)
    print('log finishes while in-air at '+str(round(on_ground_transition_time,1))+' sec')
else:
    on_ground_transition_time = float('NaN')
    print('always on ground')

if (np.amax(np.diff(airborne)) > 0.5) and (np.amin(np.diff(airborne)) < -0.5):
    if ((on_ground_transition_time - in_air_transition_time) > 0.0):
        in_air_duration = on_ground_transition_time - in_air_transition_time;
    else:
        in_air_duration = float('NaN')
else:
    in_air_duration = float('NaN')

# calculate alignment completion times
tilt_align_time_arg = np.argmax(np.diff(tilt_aligned))
tilt_align_time = status_time[tilt_align_time_arg]
yaw_align_time_arg = np.argmax(np.diff(yaw_aligned))
yaw_align_time = status_time[yaw_align_time_arg]

# calculate position aiding start times
gps_aid_time_arg = np.argmax(np.diff(using_gps))
gps_aid_time = status_time[gps_aid_time_arg]
optflow_aid_time_arg = np.argmax(np.diff(using_optflow))
optflow_aid_time = status_time[optflow_aid_time_arg]
evpos_aid_time_arg = np.argmax(np.diff(using_evpos))
evpos_aid_time = status_time[evpos_aid_time_arg]

# calculate height aiding start times
barohgt_aid_time_arg = np.argmax(np.diff(using_barohgt))
barohgt_aid_time = status_time[barohgt_aid_time_arg]
gpshgt_aid_time_arg = np.argmax(np.diff(using_gpshgt))
gpshgt_aid_time = status_time[gpshgt_aid_time_arg]
rnghgt_aid_time_arg = np.argmax(np.diff(using_rnghgt))
rnghgt_aid_time = status_time[rnghgt_aid_time_arg]
evhgt_aid_time_arg = np.argmax(np.diff(using_evhgt))
evhgt_aid_time = status_time[evhgt_aid_time_arg]

# calculate magnetometer aiding start times
using_magyaw_time_arg = np.argmax(np.diff(using_magyaw))
using_magyaw_time = status_time[using_magyaw_time_arg]
using_mag3d_time_arg = np.argmax(np.diff(using_mag3d))
using_mag3d_time = status_time[using_mag3d_time_arg]
using_magdecl_time_arg = np.argmax(np.diff(using_magdecl))
using_magdecl_time = status_time[using_magdecl_time_arg]

# control mode summary plot A
plt.figure(9,figsize=(20,13))

# subplot for alignment completion
plt.subplot(4,1,1)
plt.title('EKF Control Status - Figure A')
plt.plot(status_time,tilt_aligned,'b')
plt.plot(status_time,yaw_aligned,'r')
plt.ylim(-0.1, 1.1)
plt.ylabel('aligned')
plt.grid()
if np.amin(tilt_aligned) > 0:
    plt.text(tilt_align_time, 0.5, 'no pre-arm data - cannot calculate alignment completion times', fontsize=12, horizontalalignment='left', verticalalignment='center',color='black')
else:
    plt.text(tilt_align_time, 0.33, 'tilt alignment at '+str(round(tilt_align_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
    plt.text(yaw_align_time, 0.67, 'yaw alignment at '+str(round(tilt_align_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')

# subplot for position aiding
plt.subplot(4,1,2)
plt.plot(status_time,using_gps,'b')
plt.plot(status_time,using_optflow,'r')
plt.plot(status_time,using_evpos,'g')
plt.ylim(-0.1, 1.1)
plt.ylabel('pos aiding')
plt.grid()

if np.amin(using_gps) > 0:
    plt.text(gps_aid_time, 0.25, 'no pre-arm data - cannot calculate GPS aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
elif np.amax(using_gps) > 0:
    plt.text(gps_aid_time, 0.25, 'GPS aiding at '+str(round(gps_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')

if np.amin(using_optflow) > 0:
    plt.text(optflow_aid_time, 0.50, 'no pre-arm data - cannot calculate optical flow aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')
elif np.amax(using_optflow) > 0:
    plt.text(optflow_aid_time, 0.50, 'optical flow aiding at '+str(round(optflow_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')

if np.amin(using_evpos) > 0:
    plt.text(evpos_aid_time, 0.75, 'no pre-arm data - cannot calculate external vision aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')
elif np.amax(using_evpos) > 0:
    plt.text(evpos_aid_time, 0.75, 'external vision aiding at '+str(round(evpos_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')

# subplot for height aiding
plt.subplot(4,1,3)
plt.plot(status_time,using_barohgt,'b')
plt.plot(status_time,using_gpshgt,'r')
plt.plot(status_time,using_rnghgt,'g')
plt.plot(status_time,using_evhgt,'c')
plt.ylim(-0.1, 1.1)
plt.ylabel('hgt aiding')
plt.grid()

if np.amin(using_barohgt) > 0:
    plt.text(barohgt_aid_time, 0.2, 'no pre-arm data - cannot calculate Baro aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
elif np.amax(using_barohgt) > 0:
    plt.text(barohgt_aid_time, 0.2, 'Baro aiding at '+str(round(gps_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')

if np.amin(using_gpshgt) > 0:
    plt.text(gpshgt_aid_time, 0.4, 'no pre-arm data - cannot calculate GPS aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')
elif np.amax(using_gpshgt) > 0:
    plt.text(gpshgt_aid_time, 0.4, 'GPS aiding at '+str(round(gpshgt_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')

if np.amin(using_rnghgt) > 0:
    plt.text(rnghgt_aid_time, 0.6, 'no pre-arm data - cannot calculate rangfinder aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')
elif np.amax(using_rnghgt) > 0:
    plt.text(rnghgt_aid_time, 0.6, 'rangefinder aiding at '+str(round(rnghgt_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')

if np.amin(using_evhgt) > 0:
    plt.text(evhgt_aid_time, 0.8, 'no pre-arm data - cannot calculate external vision aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='c')
elif np.amax(using_evhgt) > 0:
    plt.text(evhgt_aid_time, 0.8, 'external vision aiding at '+str(round(evhgt_aid_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='c')

# subplot for magnetometer aiding
plt.subplot(4,1,4)
plt.plot(status_time,using_magyaw,'b')
plt.plot(status_time,using_mag3d,'r')
plt.plot(status_time,using_magdecl,'g')
plt.ylim(-0.1, 1.1)
plt.ylabel('mag aiding')
plt.xlabel('time (sec)')
plt.grid()

if np.amin(using_magyaw) > 0:
    plt.text(using_magyaw_time, 0.25, 'no pre-arm data - cannot calculate magnetic yaw aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
elif np.amax(using_magyaw) > 0:
    plt.text(using_magyaw_time, 0.25, 'magnetic yaw aiding at '+str(round(using_magyaw_time,1))+' sec', fontsize=12, horizontalalignment='right', verticalalignment='center',color='b')

if np.amin(using_mag3d) > 0:
    plt.text(using_mag3d_time, 0.50, 'no pre-arm data - cannot calculate 3D magnetoemter aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')
elif np.amax(using_mag3d) > 0:
    plt.text(using_mag3d_time, 0.50, 'magnetometer 3D aiding at '+str(round(using_mag3d_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='r')

if np.amin(using_magdecl) > 0:
    plt.text(using_magdecl_time, 0.75, 'no pre-arm data - cannot magnetic declination aiding start time', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')
elif np.amax(using_magdecl) > 0:
    plt.text(using_magdecl_time, 0.75, 'magnetic declination aiding at '+str(round(using_magdecl_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='g')

pp.savefig()

# control mode summary plot B
plt.figure(10,figsize=(20,13))

# subplot for airborne status
plt.subplot(2,1,1)
plt.title('EKF Control Status - Figure B')
plt.plot(status_time,airborne,'b')
plt.ylim(-0.1, 1.1)
plt.ylabel('airborne')
plt.grid()

if np.amax(np.diff(airborne)) < 0.5:
    plt.text(in_air_transition_time, 0.67, 'ground to air transition not detected', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
else:
    plt.text(in_air_transition_time, 0.67, 'in-air at '+str(round(in_air_transition_time,1))+' sec', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')

if np.amin(np.diff(airborne)) > -0.5:
    plt.text(on_ground_transition_time, 0.33, 'air to ground transition not detected', fontsize=12, horizontalalignment='left', verticalalignment='center',color='b')
else:
    plt.text(on_ground_transition_time, 0.33, 'on-ground at '+str(round(on_ground_transition_time,1))+' sec', fontsize=12, horizontalalignment='right', verticalalignment='center',color='b')

if in_air_duration > 0.0:
    plt.text((in_air_transition_time+on_ground_transition_time)/2, 0.5, 'duration = '+str(round(in_air_duration,1))+' sec', fontsize=12, horizontalalignment='center', verticalalignment='center',color='b')

# subplot for wind estimation status
plt.subplot(2,1,2)
plt.plot(status_time,estimating_wind,'b')
plt.ylim(-0.1, 1.1)
plt.ylabel('estimating wind')
plt.xlabel('time (sec)')
plt.grid()

pp.savefig()

# innovation_check_flags summary
plt.figure(11,figsize=(20,13))
# 0 - true if velocity observations have been rejected
# 1 - true if horizontal position observations have been rejected
# 2 - true if true if vertical position observations have been rejected
# 3 - true if the X magnetometer observation has been rejected
# 4 - true if the Y magnetometer observation has been rejected
# 5 - true if the Z magnetometer observation has been rejected
# 6 - true if the yaw observation has been rejected
# 7 - true if the airspeed observation has been rejected
# 8 - true if the height above ground observation has been rejected
# 9 - true if the X optical flow observation has been rejected
# 10 - true if the Y optical flow observation has been rejected
vel_innov_fail = ((2**0 & estimator_status['innovation_check_flags']) > 0)*1
posh_innov_fail = ((2**1 & estimator_status['innovation_check_flags']) > 0)*1
posv_innov_fail = ((2**2 & estimator_status['innovation_check_flags']) > 0)*1
magx_innov_fail = ((2**3 & estimator_status['innovation_check_flags']) > 0)*1
magy_innov_fail = ((2**4 & estimator_status['innovation_check_flags']) > 0)*1
magz_innov_fail = ((2**5 & estimator_status['innovation_check_flags']) > 0)*1
yaw_innov_fail = ((2**6 & estimator_status['innovation_check_flags']) > 0)*1
tas_innov_fail = ((2**7 & estimator_status['innovation_check_flags']) > 0)*1
hagl_innov_fail = ((2**8 & estimator_status['innovation_check_flags']) > 0)*1
ofx_innov_fail = ((2**9 & estimator_status['innovation_check_flags']) > 0)*1
ofy_innov_fail = ((2**10 & estimator_status['innovation_check_flags']) > 0)*1

plt.subplot(5,1,1)
plt.title('EKF Innovation Test Fails')
plt.plot(status_time,vel_innov_fail,'b',label='vel NED')
plt.plot(status_time,posh_innov_fail,'r',label='pos NE')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.legend(loc='upper left')
plt.grid()

plt.subplot(5,1,2)
plt.plot(status_time,posv_innov_fail,'b',label='hgt absolute')
plt.plot(status_time,hagl_innov_fail,'r',label='hgt above ground')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.legend(loc='upper left')
plt.grid()

plt.subplot(5,1,3)
plt.plot(status_time,magx_innov_fail,'b',label='mag_x')
plt.plot(status_time,magy_innov_fail,'r',label='mag_y')
plt.plot(status_time,magz_innov_fail,'g',label='mag_z')
plt.plot(status_time,yaw_innov_fail,'c',label='yaw')
plt.legend(loc='upper left')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.grid()

plt.subplot(5,1,4)
plt.plot(status_time,tas_innov_fail,'b',label='airspeed')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.legend(loc='upper left')
plt.grid()

plt.subplot(5,1,5)
plt.plot(status_time,ofx_innov_fail,'b',label='flow X')
plt.plot(status_time,ofy_innov_fail,'r',label='flow Y')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.xlabel('time (sec')
plt.legend(loc='upper left')
plt.grid()

pp.savefig()

# gps_check_fail_flags summary
plt.figure(12,figsize=(20,13))
# 0 : minimum required sat count fail
# 1 : minimum required GDoP fail
# 2 : maximum allowed horizontal position error fail
# 3 : maximum allowed vertical position error fail
# 4 : maximum allowed speed error fail
# 5 : maximum allowed horizontal position drift fail
# 6 : maximum allowed vertical position drift fail
# 7 : maximum allowed horizontal speed fail
# 8 : maximum allowed vertical velocity discrepancy fail
nsat_fail = ((2**0 & estimator_status['gps_check_fail_flags']) > 0)*1
gdop_fail = ((2**1 & estimator_status['gps_check_fail_flags']) > 0)*1
herr_fail = ((2**2 & estimator_status['gps_check_fail_flags']) > 0)*1
verr_fail = ((2**3 & estimator_status['gps_check_fail_flags']) > 0)*1
serr_fail = ((2**4 & estimator_status['gps_check_fail_flags']) > 0)*1
hdrift_fail = ((2**5 & estimator_status['gps_check_fail_flags']) > 0)*1
vdrift_fail = ((2**6 & estimator_status['gps_check_fail_flags']) > 0)*1
hspd_fail = ((2**7 & estimator_status['gps_check_fail_flags']) > 0)*1
veld_diff_fail = ((2**8 & estimator_status['gps_check_fail_flags']) > 0)*1

plt.subplot(2,1,1)
plt.title('GPS Direct Output Check Failures')
plt.plot(status_time,nsat_fail,'b',label='N sats')
plt.plot(status_time,gdop_fail,'r',label='GDOP')
plt.plot(status_time,herr_fail,'g',label='horiz pos error')
plt.plot(status_time,verr_fail,'c',label='vert pos error')
plt.plot(status_time,serr_fail,'m',label='speed error')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.legend(loc='upper right')
plt.grid()

plt.subplot(2,1,2)
plt.title('GPS Derived Output Check Failures')
plt.plot(status_time,hdrift_fail,'b',label='horiz drift')
plt.plot(status_time,vdrift_fail,'r',label='vert drift')
plt.plot(status_time,hspd_fail,'g',label='horiz speed')
plt.plot(status_time,veld_diff_fail,'c',label='vert vel inconsistent')
plt.ylim(-0.1, 1.1)
plt.ylabel('failed')
plt.xlabel('time (sec')
plt.legend(loc='upper right')
plt.grid()

pp.savefig()

# filter reported accuracy
plt.figure(13,figsize=(20,13))
plt.title('Reported Accuracy')
plt.plot(status_time,estimator_status['pos_horiz_accuracy'],'b',label='horizontal')
plt.plot(status_time,estimator_status['pos_vert_accuracy'],'r',label='vertical')
plt.ylabel('accuracy (m)')
plt.xlabel('time (sec')
plt.legend(loc='upper right')
plt.grid()

pp.savefig()

# Plot the EKF IMU vibration metrics
plt.figure(14,figsize=(20,13))

vibe_coning_max_arg = np.argmax(estimator_status['vibe[0]'])
vibe_coning_max_time = status_time[vibe_coning_max_arg]
vibe_coning_max = np.amax(estimator_status['vibe[0]'])

vibe_hf_dang_max_arg = np.argmax(estimator_status['vibe[1]'])
vibe_hf_dang_max_time = status_time[vibe_hf_dang_max_arg]
vibe_hf_dang_max = np.amax(estimator_status['vibe[1]'])

vibe_hf_dvel_max_arg = np.argmax(estimator_status['vibe[2]'])
vibe_hf_dvel_max_time = status_time[vibe_hf_dvel_max_arg]
vibe_hf_dvel_max = np.amax(estimator_status['vibe[2]'])

plt.subplot(3,1,1)
plt.plot(1e-6*estimator_status['timestamp'] , 1000.0 * estimator_status['vibe[0]'],'b')
plt.title('IMU Vibration Metrics')
plt.ylabel('Del Ang Coning (mrad)')
plt.grid()
plt.text(vibe_coning_max_time, 1000.0 * vibe_coning_max, 'max='+str(round(1000.0 * vibe_coning_max,5)), fontsize=12, horizontalalignment='left', verticalalignment='top')

plt.subplot(3,1,2)
plt.plot(1e-6*estimator_status['timestamp'] , 1000.0 * estimator_status['vibe[1]'],'b')
plt.ylabel('HF Del Ang (mrad)')
plt.grid()
plt.text(vibe_hf_dang_max_time, 1000.0 * vibe_hf_dang_max, 'max='+str(round(1000.0 * vibe_hf_dang_max,3)), fontsize=12, horizontalalignment='left', verticalalignment='top')

plt.subplot(3,1,3)
plt.plot(1e-6*estimator_status['timestamp'] , estimator_status['vibe[2]'],'b')
plt.ylabel('HF Del Vel (m/s)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(vibe_hf_dvel_max_time, vibe_hf_dvel_max, 'max='+str(round(vibe_hf_dvel_max,4)), fontsize=12, horizontalalignment='left', verticalalignment='top')

pp.savefig()

# Plot the EKF output observer tracking errors
plt.figure(15,figsize=(20,13))

ang_track_err_max_arg = np.argmax(ekf2_innovations['output_tracking_error[0]'])
ang_track_err_max_time = innov_time[ang_track_err_max_arg]
ang_track_err_max = np.amax(ekf2_innovations['output_tracking_error[0]'])

vel_track_err_max_arg = np.argmax(ekf2_innovations['output_tracking_error[1]'])
vel_track_err_max_time = innov_time[vel_track_err_max_arg]
vel_track_err_max = np.amax(ekf2_innovations['output_tracking_error[1]'])

pos_track_err_max_arg = np.argmax(ekf2_innovations['output_tracking_error[2]'])
pos_track_err_max_time = innov_time[pos_track_err_max_arg]
pos_track_err_max = np.amax(ekf2_innovations['output_tracking_error[2]'])

plt.subplot(3,1,1)
plt.plot(1e-6*ekf2_innovations['timestamp'] , 1e3*ekf2_innovations['output_tracking_error[0]'],'b')
plt.title('Output Observer Tracking Error Magnitudes')
plt.ylabel('angles (mrad)')
plt.grid()
plt.text(ang_track_err_max_time, 1e3 * ang_track_err_max, 'max='+str(round(1e3 * ang_track_err_max,2)), fontsize=12, horizontalalignment='left', verticalalignment='top')

plt.subplot(3,1,2)
plt.plot(1e-6*ekf2_innovations['timestamp'] , ekf2_innovations['output_tracking_error[1]'],'b')
plt.ylabel('velocity (m/s)')
plt.grid()
plt.text(vel_track_err_max_time, vel_track_err_max, 'max='+str(round(vel_track_err_max,2)), fontsize=12, horizontalalignment='left', verticalalignment='top')

plt.subplot(3,1,3)
plt.plot(1e-6*ekf2_innovations['timestamp'] , ekf2_innovations['output_tracking_error[2]'],'b')
plt.ylabel('position (m)')
plt.xlabel('time (sec)')
plt.grid()
plt.text(pos_track_err_max_time, pos_track_err_max, 'max='+str(round(pos_track_err_max,2)), fontsize=12, horizontalalignment='left', verticalalignment='top')

pp.savefig()

# Plot the EKF wind estimates
plt.figure(16,figsize=(20,13))

plt.subplot(2,1,1)
plt.plot(1e-6*estimator_status['timestamp'] , estimator_status['states[22]'],'b')
plt.title('Wind Velocity Estimates')
plt.ylabel('North (m/s)')
plt.xlabel('time (sec)')
plt.grid()

plt.subplot(2,1,2)
plt.plot(1e-6*estimator_status['timestamp'] , estimator_status['states[23]'],'b')
plt.ylabel('East (m/s)')
plt.xlabel('time (sec)')
plt.grid()

pp.savefig()

# close the pdf file
pp.close()

# don't display to screen
#plt.show()

# clase all figures
plt.close("all")

# Do some automated analysis of the status data

# find a late/early index range from 5 sec after in_air_transtion_time to 5 sec before on-ground transition time for mag and optical flow checks to avoid false positives
# this can be used to prevent false positives for sensors adversely affected by close proximity to the ground
late_start_index = np.argmin(status_time[np.where(status_time > (in_air_transition_time+5.0))])
early_end_index = np.argmax(status_time[np.where(status_time < (on_ground_transition_time-5.0))])
num_valid_values_trimmed = (early_end_index - late_start_index +1)

# normal index range is defined by the flight duration
start_index = np.argmin(status_time[np.where(status_time > in_air_transition_time)])
end_index = np.argmax(status_time[np.where(status_time < on_ground_transition_time)])
num_valid_values = (end_index - start_index +1)

# also find the start and finish indexes for the innovation data
innov_late_start_index = np.argmin(innov_time[np.where(innov_time > (in_air_transition_time+5.0))])
innov_early_end_index = np.argmax(innov_time[np.where(innov_time < (on_ground_transition_time-5.0))])
innov_num_valid_values_trimmed = (innov_early_end_index - innov_late_start_index +1)
innov_start_index = np.argmin(innov_time[np.where(innov_time > in_air_transition_time)])
innov_end_index = np.argmax(innov_time[np.where(innov_time < on_ground_transition_time)])
innov_num_valid_values = (innov_end_index - innov_start_index +1)

# define dictionary of test results and descriptions
test_results = {
'master_status':['Pass','Master check status which can be either Pass Warning or Fail. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'mag_sensor_status':['Pass','Magnetometer sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'yaw_sensor_status':['Pass','Yaw sensor check summary. This sensor data can be sourced from the magnetometer or an external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'vel_sensor_status':['Pass','Velocity sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'pos_sensor_status':['Pass','Position sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'hgt_sensor_status':['Pass','Height sensor check summary. This sensor data can be sourced from either Baro, GPS, range fidner or external vision system. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'hagl_sensor_status':['Pass','Height above ground sensor check summary. This sensor data is normally sourced from a rangefinder sensor. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'tas_sensor_status':['Pass','Airspeed sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'imu_sensor_status':['Pass','IMU sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'flow_sensor_status':['Pass','Optical Flow sensor check summary. A Fail result indicates a significant error that caused a significant reduction in vehicle navigation performance was detected. A Warning result indicates that error levels higher than normal were detected but these errors did not significantly impact navigation performance. A Pass result indicates that no amonalies were detected and no further investigation is required'],
'mag_percentage_red':[float('NaN'),'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 1.0.'],
'mag_percentage_amber':[float('NaN'),'The percentage of in-flight consolidated magnetic field sensor innovation consistency test values > 0.5.'],
'magx_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the X-axis magnetic field sensor innovation consistency test.'],
'magy_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the Y-axis magnetic field sensor innovation consistency test.'],
'magz_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the Z-axis magnetic field sensor innovation consistency test.'],
'yaw_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the yaw sensor innovation consistency test.'],
'mag_test_max':[float('NaN'),'The maximum in-flight value of the magnetic field sensor innovation consistency test ratio.'],
'mag_test_mean':[float('NaN'),'The mean in-flight value of the magnetic field sensor innovation consistency test ratio.'],
'vel_percentage_red':[float('NaN'),'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 1.0.'],
'vel_percentage_amber':[float('NaN'),'The percentage of in-flight velocity sensor consolidated innovation consistency test values > 0.5.'],
'vel_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
'vel_test_max':[float('NaN'),'The maximum in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
'vel_test_mean':[float('NaN'),'The mean in-flight value of the velocity sensor consolidated innovation consistency test ratio.'],
'pos_percentage_red':[float('NaN'),'The percentage of in-flight position sensor consolidated innovation consistency test values > 1.0.'],
'pos_percentage_amber':[float('NaN'),'The percentage of in-flight position sensor consolidated innovation consistency test values > 0.5.'],
'pos_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the velocity sensor consolidated innovation consistency test.'],
'pos_test_max':[float('NaN'),'The maximum in-flight value of the position sensor consolidated innovation consistency test ratio.'],
'pos_test_mean':[float('NaN'),'The mean in-flight value of the position sensor consolidated innovation consistency test ratio.'],
'hgt_percentage_red':[float('NaN'),'The percentage of in-flight height sensor innovation consistency test values > 1.0.'],
'hgt_percentage_amber':[float('NaN'),'The percentage of in-flight height sensor innovation consistency test values > 0.5.'],
'hgt_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the height sensor innovation consistency test.'],
'hgt_test_max':[float('NaN'),'The maximum in-flight value of the height sensor innovation consistency test ratio.'],
'hgt_test_mean':[float('NaN'),'The mean in-flight value of the height sensor innovation consistency test ratio.'],
'tas_percentage_red':[float('NaN'),'The percentage of in-flight airspeed sensor innovation consistency test values > 1.0.'],
'tas_percentage_amber':[float('NaN'),'The percentage of in-flight airspeed sensor innovation consistency test values > 0.5.'],
'tas_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the airspeed sensor innovation consistency test.'],
'tas_test_max':[float('NaN'),'The maximum in-flight value of the airspeed sensor innovation consistency test ratio.'],
'tas_test_mean':[float('NaN'),'The mean in-flight value of the airspeed sensor innovation consistency test ratio.'],
'hagl_percentage_red':[float('NaN'),'The percentage of in-flight height above ground sensor innovation consistency test values > 1.0.'],
'hagl_percentage_amber':[float('NaN'),'The percentage of in-flight height above ground sensor innovation consistency test values > 0.5.'],
'hagl_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the height above ground sensor innovation consistency test.'],
'hagl_test_max':[float('NaN'),'The maximum in-flight value of the height above ground sensor innovation consistency test ratio.'],
'hagl_test_mean':[float('NaN'),'The mean in-flight value of the height above ground sensor innovation consistency test ratio.'],
'ofx_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the optical flow sensor X-axis innovation consistency test.'],
'ofy_fail_percentage':[float('NaN'),'The percentage of in-flight recorded failure events for the optical flow sensor Y-axis innovation consistency test.'],
'filter_faults_max':[float('NaN'),'Largest recorded value of the filter internal fault bitmask. Should always be zero.'],
'imu_coning_peak':[float('NaN'),'Peak in-flight value of the IMU delta angle coning vibration metric (rad)'],
'imu_coning_mean':[float('NaN'),'Mean in-flight value of the IMU delta angle coning vibration metric (rad)'],
'imu_hfdang_peak':[float('NaN'),'Peak in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
'imu_hfdang_mean':[float('NaN'),'Mean in-flight value of the IMU delta angle high frequency vibration metric (rad)'],
'imu_hfdvel_peak':[float('NaN'),'Peak in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
'imu_hfdvel_mean':[float('NaN'),'Mean in-flight value of the IMU delta velocity high frequency vibration metric (m/s)'],
'output_obs_ang_err_peak':[float('NaN'),'Peak in-flight value of the output observer angular error (rad)'],
'output_obs_ang_err_mean':[float('NaN'),'Mean in-flight value of the output observer angular error (rad)'],
'output_obs_vel_err_peak':[float('NaN'),'Peak in-flight value of the output observer velocity error (m/s)'],
'output_obs_vel_err_mean':[float('NaN'),'Mean in-flight value of the output observer velocity error (m/s)'],
'output_obs_pos_err_peak':[float('NaN'),'Peak in-flight value of the output observer position error (m)'],
'output_obs_pos_err_mean':[float('NaN'),'Mean in-flight value of the output observer position error (m)'],
'tilt_align_time':[float('NaN'),'The time in seconds measured from startup that the EKF completed the tilt alignment. A nan value indicates that the alignment had completed before logging started or alignment did not complete.'],
'yaw_align_time':[float('NaN'),'The time in seconds measured from startup that the EKF completed the yaw alignment.'],
'in_air_transition_time':[round(in_air_transition_time,1),'The time in seconds measured from startup that the EKF transtioned into in-air mode. Set to a nan if a transition event is not detected.'],
'on_ground_transition_time':[round(on_ground_transition_time,1),'The time in seconds measured from startup  that the EKF transitioned out of in-air mode. Set to a nan if a transition event is not detected.'],
}

# generate test metadata

# reduction of innovation message data
if (innov_early_end_index > (innov_late_start_index + 100)):
    # Output Observer Tracking Errors
    temp = np.amax(ekf2_innovations['output_tracking_error[0]'][innov_late_start_index:innov_early_end_index])
    if (temp > 0.0):
        test_results['output_obs_ang_err_peak'][0] = temp
        test_results['output_obs_ang_err_mean'][0] = np.mean(ekf2_innovations['output_tracking_error[0]'][innov_late_start_index:innov_early_end_index])
    temp = np.amax(ekf2_innovations['output_tracking_error[1]'][innov_late_start_index:innov_early_end_index])
    if (temp > 0.0):
        test_results['output_obs_vel_err_peak'][0] = temp
        test_results['output_obs_vel_err_mean'][0] = np.mean(ekf2_innovations['output_tracking_error[1]'][innov_late_start_index:innov_early_end_index])
    temp = np.amax(ekf2_innovations['output_tracking_error[2]'][innov_late_start_index:innov_early_end_index])
    if (temp > 0.0):
        test_results['output_obs_pos_err_peak'][0] = temp
        test_results['output_obs_pos_err_mean'][0] = np.mean(ekf2_innovations['output_tracking_error[2]'][innov_late_start_index:innov_early_end_index])

# reduction of status message data
if (early_end_index > (late_start_index + 100)):
    # IMU vibration checks
    temp = np.amax(estimator_status['vibe[0]'][late_start_index:early_end_index])
    if (temp > 0.0):
        test_results['imu_coning_peak'][0] = temp
        test_results['imu_coning_mean'][0] = np.mean(estimator_status['vibe[0]'][late_start_index:early_end_index])
    temp = np.amax(estimator_status['vibe[1]'][late_start_index:early_end_index])
    if (temp > 0.0):
        test_results['imu_hfdang_peak'][0] = temp
        test_results['imu_hfdang_mean'][0] = np.mean(estimator_status['vibe[1]'][late_start_index:early_end_index])
    temp = np.amax(estimator_status['vibe[2]'][late_start_index:early_end_index])
    if (temp > 0.0):
        test_results['imu_hfdvel_peak'][0] = temp
        test_results['imu_hfdvel_mean'][0] = np.mean(estimator_status['vibe[2]'][late_start_index:early_end_index])

    # Magnetometer Sensor Checks
    if (np.amax(yaw_aligned) > 0.5):
        mag_num_red = (estimator_status['mag_test_ratio'][start_index:end_index] > 1.0).sum()
        mag_num_amber = (estimator_status['mag_test_ratio'][start_index:end_index] > 0.5).sum() - mag_num_red
        test_results['mag_percentage_red'][0] = 100.0 * mag_num_red / num_valid_values_trimmed
        test_results['mag_percentage_amber'][0] = 100.0 * mag_num_amber / num_valid_values_trimmed
        test_results['mag_test_max'][0] = np.amax(estimator_status['mag_test_ratio'][late_start_index:early_end_index])
        test_results['mag_test_mean'][0] = np.mean(estimator_status['mag_test_ratio'][start_index:end_index])
        test_results['magx_fail_percentage'][0] = 100.0 * (magx_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed
        test_results['magy_fail_percentage'][0] = 100.0 * (magy_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed
        test_results['magz_fail_percentage'][0] = 100.0 * (magz_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed
        test_results['yaw_fail_percentage'][0] = 100.0 * (yaw_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed

    # Velocity Sensor Checks
    if (np.amax(using_gps) > 0.5):
        vel_num_red = (estimator_status['vel_test_ratio'][start_index:end_index] > 1.0).sum()
        vel_num_amber = (estimator_status['vel_test_ratio'][start_index:end_index] > 0.5).sum() - vel_num_red
        test_results['vel_percentage_red'][0] = 100.0 * vel_num_red / num_valid_values
        test_results['vel_percentage_amber'][0] = 100.0 * vel_num_amber / num_valid_values
        test_results['vel_test_max'][0] = np.amax(estimator_status['vel_test_ratio'][start_index:end_index])
        test_results['vel_test_mean'][0] = np.mean(estimator_status['vel_test_ratio'][start_index:end_index])
        test_results['vel_fail_percentage'][0] = 100.0 * (vel_innov_fail[start_index:end_index] > 0.5).sum() / num_valid_values

    # Position Sensor Checks
    if ((np.amax(using_gps) > 0.5) or (np.amax(using_evpos) > 0.5)):
        pos_num_red = (estimator_status['pos_test_ratio'][start_index:end_index] > 1.0).sum()
        pos_num_amber = (estimator_status['pos_test_ratio'][start_index:end_index] > 0.5).sum() - pos_num_red
        test_results['pos_percentage_red'][0] = 100.0 * pos_num_red / num_valid_values
        test_results['pos_percentage_amber'][0] = 100.0 * pos_num_amber / num_valid_values
        test_results['pos_test_max'][0] = np.amax(estimator_status['pos_test_ratio'][start_index:end_index])
        test_results['pos_test_mean'][0] = np.mean(estimator_status['pos_test_ratio'][start_index:end_index])
        test_results['pos_fail_percentage'][0] = 100.0 * (posh_innov_fail[start_index:end_index] > 0.5).sum() / num_valid_values

    # Height Sensor Checks
    hgt_num_red = (estimator_status['hgt_test_ratio'][late_start_index:early_end_index] > 1.0).sum()
    hgt_num_amber = (estimator_status['hgt_test_ratio'][late_start_index:early_end_index] > 0.5).sum() - hgt_num_red
    test_results['hgt_percentage_red'][0] = 100.0 * hgt_num_red / num_valid_values_trimmed
    test_results['hgt_percentage_amber'][0] = 100.0 * hgt_num_amber / num_valid_values_trimmed
    test_results['hgt_test_max'][0] = np.amax(estimator_status['hgt_test_ratio'][late_start_index:early_end_index])
    test_results['hgt_test_mean'][0] = np.mean(estimator_status['hgt_test_ratio'][late_start_index:early_end_index])
    test_results['hgt_fail_percentage'][0] = 100.0 * (posv_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed

    # Airspeed Sensor Checks
    if (tas_test_max > 0.0):
        tas_num_red = (estimator_status['tas_test_ratio'][start_index:end_index] > 1.0).sum()
        tas_num_amber = (estimator_status['tas_test_ratio'][start_index:end_index] > 0.5).sum() - tas_num_red
        test_results['tas_percentage_red'][0] = 100.0 * tas_num_red / num_valid_values
        test_results['tas_percentage_amber'][0] = 100.0 * tas_num_amber / num_valid_values
        test_results['tas_test_max'][0] = np.amax(estimator_status['tas_test_ratio'][start_index:end_index])
        test_results['tas_test_mean'][0] = np.mean(estimator_status['tas_test_ratio'][start_index:end_index])
        test_results['tas_fail_percentage'][0] = 100.0 * (tas_innov_fail[start_index:end_index] > 0.5).sum() / num_valid_values

    # HAGL Sensor Checks
    if (hagl_test_max > 0.0):
        hagl_num_red = (estimator_status['hagl_test_ratio'][start_index:end_index] > 1.0).sum()
        hagl_num_amber = (estimator_status['hagl_test_ratio'][start_index:end_index] > 0.5).sum() - hagl_num_red
        test_results['hagl_percentage_red'][0] = 100.0 * hagl_num_red / num_valid_values
        test_results['hagl_percentage_amber'][0] = 100.0 * hagl_num_amber / num_valid_values
        test_results['hagl_test_max'][0] = np.amax(estimator_status['hagl_test_ratio'][start_index:end_index])
        test_results['hagl_test_mean'][0] = np.mean(estimator_status['hagl_test_ratio'][start_index:end_index])
        test_results['hagl_fail_percentage'][0] = 100.0 * (hagl_innov_fail[start_index:end_index] > 0.5).sum() / num_valid_values

    # optical flow sensor checks
    if (np.amax(using_optflow) > 0.5):
        test_results['ofx_fail_percentage'][0] = 100.0 * (ofx_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed
        test_results['ofy_fail_percentage'][0] = 100.0 * (ofy_innov_fail[late_start_index:early_end_index] > 0.5).sum() / num_valid_values_trimmed

# Check for internal filter nummerical faults
test_results['filter_faults_max'][0] = np.amax(estimator_status['filter_fault_flags'])

# TODO - process the following bitmask's when they have been properly documented in the uORB topic
#estimator_status['health_flags']
#estimator_status['timeout_flags']

# calculate a master status - Fail, Warning, Pass
# get the dictionary of fail and warning test thresholds from a csv file
filename = "check_level_dict.csv"
file = open(filename)
check_levels = { }
for line in file:
    x = line.split(",")
    a=x[0]
    b=x[1]
    check_levels[a]=float(b)
file.close()

# print out the check levels
print('Using test criteria loaded from '+filename)
#for N in check_levels:
#    val = check_levels.get(N)
#    print(N+' = '+str(val), end='\n')

# check test results against levels to provide a master status

# check for warnings
if (test_results.get('mag_percentage_amber')[0] > check_levels.get('mag_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['mag_sensor_status'][0] = 'Warning'
if (test_results.get('vel_percentage_amber')[0] > check_levels.get('vel_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['vel_sensor_status'][0] = 'Warning'
if (test_results.get('pos_percentage_amber')[0] > check_levels.get('pos_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['pos_sensor_status'][0] = 'Warning'
if (test_results.get('hgt_percentage_amber')[0] > check_levels.get('hgt_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['hgt_sensor_status'][0] = 'Warning'
if (test_results.get('hagl_percentage_amber')[0] > check_levels.get('hagl_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['hagl_sensor_status'][0] = 'Warning'
if (test_results.get('tas_percentage_amber')[0] > check_levels.get('tas_amber_warn_pct')):
    test_results['master_status'][0] = 'Warning'
    test_results['tas_sensor_status'][0] = 'Warning'
if ((test_results.get('imu_coning_peak')[0] > check_levels.get('imu_coning_peak_warn')) or
(test_results.get('imu_coning_mean')[0] > check_levels.get('imu_coning_mean_warn')) or
(test_results.get('imu_hfdang_peak')[0] > check_levels.get('imu_hfdang_peak_warn')) or
(test_results.get('imu_hfdang_mean')[0] > check_levels.get('imu_hfdang_mean_warn')) or
(test_results.get('imu_hfdvel_peak')[0] > check_levels.get('imu_hfdvel_peak_warn')) or
(test_results.get('imu_hfdvel_mean')[0] > check_levels.get('imu_hfdvel_mean_warn'))):
    test_results['master_status'][0] = 'Warning'
    test_results['imu_sensor_status'][0] = 'Warning'
if ((test_results.get('output_obs_ang_err_peak')[0] > check_levels.get('obs_ang_err_peak_warn')) or
(test_results.get('output_obs_ang_err_mean')[0] > check_levels.get('obs_ang_err_mean_warn')) or
(test_results.get('output_obs_vel_err_peak')[0] > check_levels.get('obs_vel_err_peak_warn')) or
(test_results.get('output_obs_vel_err_mean')[0] > check_levels.get('obs_vel_err_mean_warn')) or
(test_results.get('output_obs_pos_err_peak')[0] > check_levels.get('obs_pos_err_peak_warn')) or
(test_results.get('output_obs_pos_err_mean')[0] > check_levels.get('obs_pos_err_mean_warn'))):
    test_results['master_status'][0] = 'Warning'
    test_results['imu_sensor_status'][0] = 'Warning'

# check for failures
if ((test_results.get('magx_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
(test_results.get('magy_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
(test_results.get('magz_fail_percentage')[0] > check_levels.get('mag_fail_pct')) or
(test_results.get('mag_percentage_amber')[0] > check_levels.get('mag_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['mag_sensor_status'][0] = 'Fail'
if (test_results.get('yaw_fail_percentage')[0] > check_levels.get('yaw_fail_pct')):
    test_results['master_status'][0] = 'Fail'
    test_results['yaw_sensor_status'][0] = 'Fail'
if ((test_results.get('vel_fail_percentage')[0] > check_levels.get('vel_fail_pct')) or
(test_results.get('vel_percentage_amber')[0] > check_levels.get('vel_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['vel_sensor_status'][0] = 'Fail'
if ((test_results.get('pos_fail_percentage')[0] > check_levels.get('pos_fail_pct')) or
(test_results.get('pos_percentage_amber')[0] > check_levels.get('pos_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['pos_sensor_status'][0] = 'Fail'
if ((test_results.get('hgt_fail_percentage')[0] > check_levels.get('hgt_fail_pct')) or
(test_results.get('hgt_percentage_amber')[0] > check_levels.get('hgt_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['hgt_sensor_status'][0] = 'Fail'
if ((test_results.get('tas_fail_percentage')[0] > check_levels.get('tas_fail_pct')) or
(test_results.get('tas_percentage_amber')[0] > check_levels.get('tas_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['tas_sensor_status'][0] = 'Fail'
if ((test_results.get('hagl_fail_percentage')[0] > check_levels.get('hagl_fail_pct')) or
(test_results.get('hagl_percentage_amber')[0] > check_levels.get('hagl_amber_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['hagl_sensor_status'][0] = 'Fail'
if ((test_results.get('ofx_fail_percentage')[0] > check_levels.get('flow_fail_pct')) or
(test_results.get('ofy_fail_percentage')[0] > check_levels.get('flow_fail_pct'))):
    test_results['master_status'][0] = 'Fail'
    test_results['flow_sensor_status'][0] = 'Fail'
if (test_results.get('filter_faults_max')[0] > 0):
    test_results['master_status'][0] = 'Fail'
    test_results['filter_fault_status'][0] = 'Fail'

# print master test status to console
if (test_results['master_status'][0] == 'Pass'):
    print('No anomalies detected')
elif (test_results['master_status'][0] == 'Warning'):
    print('Minor anomalies detected')
elif (test_results['master_status'][0] == 'Fail'):
    print('Major anomalies detected')

# write metadata to a .csv file
test_results_filename = ulog_file_name + ".mdat.csv"
file = open(test_results_filename,"w")

file.write("name,value,description\n")

# loop through the test results dictionary and write each entry on a separate row, with data comma separated
# save data in alphabetical order
key_list = list(test_results.keys())
key_list.sort()
for key in key_list:
    file.write(key+","+str(test_results[key][0])+","+test_results[key][1]+"\n")

file.close()

print('Test results written to ' + test_results_filename)
print('Plots saved to ' + output_plot_filename)
