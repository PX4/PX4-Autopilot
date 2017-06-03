clear all;
close all;

% load compulsory data
load '../TestData/APM/baro_data.mat';
load '../TestData/APM/gps_data.mat';
load '../TestData/APM/imu_data.mat';
load '../TestData/APM/mag_data.mat';

% load data required for optical flow replay
if exist('../TestData/APM/rng_data.mat','file') && exist('../TestData/APM/flow_data.mat','file')
    load '../TestData/APM/rng_data.mat';
    load '../TestData/APM/flow_data.mat';
else 
    rng_data = [];
    flow_data = [];
end

% oad data required for ZED camera replay
if exist('../TestData/APM/viso_data.mat','file')
    load '../TestData/APM/viso_data.mat';
else
    viso_data = [];
end

run('SetParameterDefaults.m');
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data,rng_data,flow_data,viso_data);
PlotData(output);