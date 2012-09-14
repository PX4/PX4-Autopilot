clear all
clc

% Read actuators file
fileInfo = dir('sensors_combined.bin');
fileSize = fileInfo.bytes;

fid = fopen('actuator_outputs0.bin', 'r');
elements = int64(fileSize/16*4+8)

for i=1:(elements-2)
    actuators(i, 2:18) = fread(fid,  inf, 'float');
end

% Read sensor combined file
% Type definition: Firmware/apps/uORB/topics/sensor_combined.h
% Struct: sensor_combined_s
fileInfo = dir('sensors_combined.bin');
fileSize = fileInfo.bytes;

fid = fopen('sensors_combined.bin', 'r');

for i=1:elements-2
    % timestamp
    sensors(i,1) = double(fread(fid, 1, '*uint64', 0, 'ieee-le.l64'));
    % gyro raw
    sensors(i,2:4) = fread(fid, 3, 'int16', 0, 'ieee-le');
    % gyro counter
    sensors(i,5) = fread(fid, 1, 'uint16', 0, 'ieee-le');
    % gyro in rad/s
    sensors(i,6:8) = fread(fid, 3, 'float', 0, 'ieee-le');
    
    % accelerometer raw
    sensors(i,9:11) = fread(fid, 3, 'int16', 0, 'ieee-le');
    % padding bytes
    fread(fid, 1, 'int16', 0, 'ieee-le');
    % accelerometer counter
    sensors(i,12) = fread(fid, 1, 'uint32', 0, 'ieee-le');
    % accel in m/s2
    sensors(i,13:15) = fread(fid, 3, 'float', 0, 'ieee-le');
    % accel mode
    sensors(i,16) = fread(fid, 1, 'int32', 0, 'ieee-le');
    % accel range
    sensors(i,17) = fread(fid, 1, 'float', 0, 'ieee-le');
    
    % mag raw
    sensors(i,18:20) = fread(fid, 3, 'int16', 0, 'ieee-le');
    % padding bytes
    fread(fid, 1, 'int16', 0, 'ieee-le');
    % mag in Gauss
    sensors(i,21:23) = fread(fid, 3, 'float', 0, 'ieee-le');
    % mag mode
    sensors(i,24) = fread(fid, 1, 'int32', 0, 'ieee-le');
    % mag range
    sensors(i,25) = fread(fid, 1, 'float', 0, 'ieee-le');
    % mag cuttoff freq
    sensors(i,26) = fread(fid, 1, 'float', 0, 'ieee-le');
    % mag counter
    sensors(i,27) = fread(fid, 1, 'int32', 0, 'ieee-le');
    
    % baro pressure millibar
    % baro alt meter
    % baro temp celcius
    % battery voltage
    % adc voltage (3 channels)
    sensors(i,28:34) = fread(fid, 7, 'float', 0, 'ieee-le');
    % baro counter and battery counter
    sensors(i,35:36) = fread(fid, 2, 'uint32', 0, 'ieee-le');
    % battery voltage valid flag
    sensors(i,37) = fread(fid, 1, 'uint32', 0, 'ieee-le');
    
end