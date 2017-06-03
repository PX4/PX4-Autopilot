clear all;
close all;
load '../TestData/PX4/baro_data.mat';
load '../TestData/PX4/gps_data.mat';
load '../TestData/PX4/imu_data.mat';
load '../TestData/PX4/mag_data.mat';
run('SetParameterDefaults.m');
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);
PlotData(output);