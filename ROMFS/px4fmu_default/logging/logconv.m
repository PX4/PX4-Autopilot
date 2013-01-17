% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.

% Clear everything
clc
clear all
close all

% Set the path to your sysvector.bin file here
filePath = 'sysvector.bin';

% Work around a Matlab bug (not related to PX4)
% where timestamps from 1.1.1970 do not allow to
% read the file's size
if ismac
    system('touch -t 201212121212.12 sysvector.bin');
end

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
% float bat_current - current drawn from battery at this time instant
% float bat_discharged - discharged energy in mAh
% float adc[3]; //remaining auxiliary ADC ports [volt]
% float local_position[3]; //tangent plane mapping into x,y,z [m]
% int32_t gps_raw_position[3]; //latitude [degrees] north, longitude [degrees] east, altitude above MSL [millimeter]
% float attitude[3]; //pitch, roll, yaw [rad]
% float rotMatrix[9]; //unitvectors
% float actuator_control[4]; //unitvector
% float optical_flow[4]; //roll, pitch, yaw [-1..1], thrust [0..1]
% float diff_pressure; - pressure difference in millibar
% float ind_airspeed;
% float true_airspeed;

% Definition of the logged values
logFormat{1} = struct('name', 'timestamp',             'bytes', 8, 'array', 1, 'precision', 'uint64',  'machineformat', 'ieee-le.l64');
logFormat{2} = struct('name', 'gyro',                  'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{3} = struct('name', 'accel',                 'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{4} = struct('name', 'mag',                   'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{5} = struct('name', 'baro',                  'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{6} = struct('name', 'baro_alt',              'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{7} = struct('name', 'baro_temp',             'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{8} = struct('name', 'control',               'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{9} = struct('name', 'actuators',             'bytes', 4, 'array', 8, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{10} = struct('name', 'vbat',                 'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{11} = struct('name', 'bat_current',          'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{12} = struct('name', 'bat_discharged',       'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{13} = struct('name', 'adc',                  'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{14} = struct('name', 'local_position',       'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{15} = struct('name', 'gps_raw_position',     'bytes', 4, 'array', 3, 'precision', 'uint32',  'machineformat', 'ieee-le');
logFormat{16} = struct('name', 'attitude',             'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{17} = struct('name', 'rot_matrix',           'bytes', 4, 'array', 9, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{18} = struct('name', 'vicon_position',       'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{19} = struct('name', 'actuator_control',     'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{20} = struct('name', 'optical_flow',         'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{21} = struct('name', 'diff_pressure',        'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{22} = struct('name', 'ind_airspeed',         'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
logFormat{23} = struct('name', 'true_airspeed',        'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');

% First get length of one line
columns = length(logFormat);
lineLength = 0;

for i=1:columns
    lineLength = lineLength + logFormat{i}.bytes * logFormat{i}.array;
end


if exist(filePath, 'file')
    
    fileInfo = dir(filePath);
    fileSize = fileInfo.bytes;
    
    elements = int64(fileSize./(lineLength));
    
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
    time_s = time_us*1e-6;
    time_m = time_s/60;
    
    % close the logfile
    fclose(fid);
    
    disp(['end log2matlab conversion' char(10)]);
else
    disp(['file: ' filePath ' does not exist' char(10)]);
end

%% Plot GPS RAW measurements

% Only plot GPS data if available
if cumsum(double(sysvector.gps_raw_position(200:end,1))) > 0
    figure('units','normalized','outerposition',[0 0 1 1])
    plot3(sysvector.gps_raw_position(200:end,1), sysvector.gps_raw_position(200:end,2), sysvector.gps_raw_position(200:end,3));
end


%% Plot optical flow trajectory

flow_sz = size(sysvector.timestamp);
flow_elements = flow_sz(1);

xt(1:flow_elements,1) = sysvector.timestamp(:,1); % time column [ms]


%calc dt
dt = zeros(flow_elements,1);
for i = 1:flow_elements-1
    dt(i+1,1) = double(xt(i+1,1)-xt(i,1)) * 10^(-6);   % timestep [s]
end
dt(1,1) = mean(dt);


global_speed = zeros(flow_elements,3);

%calc global speed (with rot matrix)
for i = 1:flow_elements
    rotM = [sysvector.rot_matrix(i,1:3);sysvector.rot_matrix(i,4:6);sysvector.rot_matrix(i,7:9)]';
    speedX = sysvector.optical_flow(i,3);
    speedY = sysvector.optical_flow(i,4);
    
    relSpeed = [-speedY,speedX,0];
    global_speed(i,:) = relSpeed * rotM;
end



px = zeros(flow_elements,1);
py = zeros(flow_elements,1);
distance = 0;

last_vx = 0;
last_vy = 0;
elem_cnt = 0;

% Very basic accumulation, stops on bad flow quality
for i = 1:flow_elements
    if sysvector.optical_flow(i,6) > 5
        px(i,1) = global_speed(i,1)*dt(i,1);
        py(i,1) = global_speed(i,2)*dt(i,1);
        distance = distance + norm([px(i,1) py(i,1)]);
        last_vx = px(i,1);
        last_vy = py(i,1);
    else
        px(i,1) = last_vx;
        py(i,1) = last_vy;
        last_vx = last_vx*0.95;
        last_vy = last_vy*0.95;
    end
end

px_sum = cumsum(px);
py_sum = cumsum(py);
time = cumsum(dt);

figure()
set(gca, 'Units','normal');

plot(py_sum, px_sum, '-blue', 'LineWidth',2);
axis equal;
% set title and axis captions
xlabel('X position (meters)','fontsize',14)
ylabel('Y position (meters)','fontsize',14)
% mark begin and end
hold on
plot(py_sum(1,1),px_sum(1,1),'ks','LineWidth',2,...
'MarkerEdgeColor','k',...
'MarkerFaceColor','g',...
'MarkerSize',10)
hold on
plot(py_sum(end,1),px_sum(end,1),'kv','LineWidth',2,...
'MarkerEdgeColor','k',...
'MarkerFaceColor','b',...
'MarkerSize',10)
% add total length as annotation
set(gca,'fontsize',13);
legend('Trajectory', 'START', sprintf('END\n(%.2f m, %.0f:%.0f s)', distance, time_m, time_s - time_m*60));
title('Optical Flow Position Integration', 'fontsize', 15);

figure()
plot(time, sysvector.optical_flow(:,5), 'blue');
axis([time(1,1) time(end,1) 0 (max(sysvector.optical_flow(i,5))+0.2)]);
xlabel('seconds','fontsize',14);
ylabel('m','fontsize',14);
set(gca,'fontsize',13);
title('Ultrasound Altitude', 'fontsize', 15);


figure()
plot(time, global_speed(:,2), 'red');
hold on;
plot(time, global_speed(:,1), 'blue');
legend('y velocity (m/s)', 'x velocity (m/s)');
xlabel('seconds','fontsize',14);
ylabel('m/s','fontsize',14);
set(gca,'fontsize',13);
title('Optical Flow Velocity', 'fontsize', 15);
