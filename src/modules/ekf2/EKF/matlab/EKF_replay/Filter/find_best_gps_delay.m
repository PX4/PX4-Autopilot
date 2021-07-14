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

% Loop through range of GPS time delays and capture the RMS velocity
% innovation and corresponding delay each time
for index = 1:1:21
    index
    param.fusion.gpsTimeDelay = 0.01*(index - 1);
    output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);
    gps_delay(index) = param.fusion.gpsTimeDelay;
    gps_vel_innov_rms(index) = sqrt(sum(output.innovations.vel_innov(:,1).^2) + sum(output.innovations.vel_innov(:,2).^2))/sqrt(length(output.innovations.vel_innov));
end

% plot the innvation level vs time delay
figure;
plot(gps_delay,gps_vel_innov_rms);
grid on;
xlabel('GPS delay (sec)');
ylabel('RMS velocity innovation (m/s)');

% find the smallest value and re-run the replay
param.fusion.gpsTimeDelay = gps_delay(gps_delay == min(gps_delay))

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
runIdentifier = ' : PX4 data replay ';
folder = strcat('../OutputPlots/PX4');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/PX4';
fileName = '../OutputData/PX4/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');