% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.

% Clear everything
clc
clear all
close all

% Set the path to your sysvector.bin file here
filePath = 'sysvector.bin';

%%%%%%%%%%%%%%%%%%%%%%%
% SYSTEM VECTOR
%
% //All measurements in NED frame
%
% uint64_t timestamp; //[us]
% float gyro[3]; //[rad/s]
% float accel[3]; //[m/s^2]
% float mag[3]; //[gauss] 
% float baro; //pressure [millibar]
% float baro_alt; //altitude above MSL [meter]
% float baro_temp; //[degree celcius]
% float control[4]; //roll, pitch, yaw [-1..1], thrust [0..1]
% float actuators[8]; //motor 1-8, in motor units (PWM: 1000-2000,AR.Drone: 0-512)
% float vbat; //battery voltage in [volt]
% float adc[3]; //remaining auxiliary ADC ports [volt]
% float local_position[3]; //tangent plane mapping into x,y,z [m]
% int32_t gps_raw_position[3]; //latitude [degrees] north, longitude [degrees] east, altitude above MSL [millimeter]
% float attitude[3]; //pitch, roll, yaw [rad]
% float rotMatrix[9]; //unitvectors
% float actuator_control[4]; //unitvector
% float optical_flow[4]; //roll, pitch, yaw [-1..1], thrust [0..1]

% Definition of the logged values
logFormat{1} = struct('name', 'timestamp',         'bytes', 8, 'array', 1, 'precision', 'uint64', 'machineformat', 'ieee-le.l64'); 
logFormat{2} = struct('name', 'gyro',              'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{3} = struct('name', 'accel',             'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{4} = struct('name', 'mag',               'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{5} = struct('name', 'baro',              'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{6} = struct('name', 'baro_alt',          'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{7} = struct('name', 'baro_temp',         'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{8} = struct('name', 'control',           'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{9} = struct('name', 'actuators',         'bytes', 4, 'array', 8, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{10} = struct('name', 'vbat',             'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{11} = struct('name', 'adc',              'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{12} = struct('name', 'local_position',   'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le'); 
logFormat{13} = struct('name', 'gps_raw_position', 'bytes', 4, 'array', 3, 'precision', 'uint32',  'machineformat', 'ieee-le'); 
logFormat{14} = struct('name', 'attitude',         'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{15} = struct('name', 'rot_matrix',       'bytes', 4, 'array', 9, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{16} = struct('name', 'vicon_position',   'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{17} = struct('name', 'actuator_control', 'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{18} = struct('name', 'optical_flow',     'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');


% First get length of one line
columns = length(logFormat);
lineLength = 0;

for i=1:columns
    lineLength = lineLength + logFormat{i}.bytes * logFormat{i}.array;
end


if exist(filePath, 'file')
    
    fileInfo = dir(filePath);
    fileSize = fileInfo.bytes;
    
    elements = int64(fileSize./(lineLength))
    
    fid = fopen(filePath, 'r');
    offset = 0;
    for i=1:columns
        % using fread with a skip speeds up the import drastically, do not
        % import the values one after the other
        sysvector.(genvarname(logFormat{i}.name)) = transpose(fread(...
            fid, ...
            [logFormat{i}.array, elements], [num2str(logFormat{i}.array),'*',logFormat{i}.precision,'=>',logFormat{i}.precision], ...
            lineLength - logFormat{i}.bytes*logFormat{i}.array, ...
            logFormat{i}.machineformat) ...
        );
        offset = offset + logFormat{i}.bytes*logFormat{i}.array;
        fseek(fid, offset,'bof');
    end
    
    % shot the flight time
    time_us = sysvector.timestamp(end) - sysvector.timestamp(1);
    time_s = time_us*1e-6
    time_m = time_s/60
    
    % close the logfile
    fclose(fid);
    
    disp(['end log2matlab conversion' char(10)]);
else
    disp(['file: ' filePath ' does not exist' char(10)]);
end


