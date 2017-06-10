clear all;
close all;

% add required paths
addpath('../Common');

% load test data
load '../TestData/PX4/baro_data.mat';
load '../TestData/PX4/gps_data.mat';
load '../TestData/PX4/imu_data.mat';
load '../TestData/PX4/mag_data.mat';

% set parameters to default values
run('SetParameterDefaults.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
PlotData(output);

% save output data
folder = '../OutputData';
fileName = '../OutputData/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');