clear all;
close all;

% add required paths
addpath('../Common');

% load compulsory data
load '../TestData/APM/baro_data.mat';
load '../TestData/APM/gps_data.mat';
load '../TestData/APM/imu_data.mat';
load '../TestData/APM/mag_data.mat';

% load optional data required for optical flow replay
if exist('../TestData/APM/rng_data.mat','file') && exist('../TestData/APM/flow_data.mat','file')
    load '../TestData/APM/rng_data.mat';
    load '../TestData/APM/flow_data.mat';
else
    rng_data = [];
    flow_data = [];
end

% load optional data required for ZED camera replay
if exist('../TestData/APM/viso_data.mat','file')
    load '../TestData/APM/viso_data.mat';
else
    viso_data = [];
end

% load default parameters
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data,rng_data,flow_data,viso_data);

% generate and save output plots
runIdentifier = ' : APM data replay ';
folder = strcat('../OutputPlots/APM');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/APM';
fileName = '../OutputData/APM/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');
