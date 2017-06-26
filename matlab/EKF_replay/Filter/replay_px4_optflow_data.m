clear all;
close all;

% add required paths
addpath('../Common');

% load test data
load '../TestData/PX4_optical_flow/baro_data.mat';
load '../TestData/PX4_optical_flow/gps_data.mat';
load '../TestData/PX4_optical_flow/imu_data.mat';
load '../TestData/PX4_optical_flow/mag_data.mat';
load '../TestData/PX4_optical_flow/rng_data.mat';
load '../TestData/PX4_optical_flow/flow_data.mat';

% set parameters to default values
run('SetParameterDefaults.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data,rng_data,flow_data);

% generate and save output plots
runIdentifier = ' : PX4 optical flow replay ';
folder = strcat('../OutputPlots/PX4_optical_flow');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/PX4_optical_flow';
fileName = '../OutputData/PX4_optical_flow/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');