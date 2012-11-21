clear all
close all

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

%myPath = '..\LOG30102012\session0002\';    %set relative path here
myPath = '.\';                             
myFile = 'sysvector.bin';
filePath = strcat(myPath,myFile);

if exist(filePath, 'file')
    fileInfo = dir(filePath);
    fileSize = fileInfo.bytes;
    
    fid = fopen(filePath, 'r');
    elements = int64(fileSize./(8+(3+3+3+1+1+1+4+8+1+3+3+3+3+9+6+4+6)*4))
    
    for i=1:elements       
        % timestamp
        sensors(i,1) = double(fread(fid, 1, '*uint64', 0, 'ieee-le.l64'));
        
        % gyro (3 channels)
        sensors(i,2:4) = fread(fid, 3, 'float', 0, 'ieee-le');
       
        % accelerometer (3 channels)
        sensors(i,5:7) = fread(fid, 3, 'float', 0, 'ieee-le');
      
        % mag (3 channels)
        sensors(i,8:10) = fread(fid, 3, 'float', 0, 'ieee-le');
       
        % baro pressure
        sensors(i,11) = fread(fid, 1, 'float', 0, 'ieee-le');
        
        % baro alt
        sensors(i,12) = fread(fid, 1, 'float', 0, 'ieee-le');
         
        % baro temp
        sensors(i,13) = fread(fid, 1, 'float', 0, 'ieee-le');
        
        % actuator control (4 channels) 
        sensors(i,14:17) = fread(fid, 4, 'float', 0, 'ieee-le');
        
        % actuator outputs (8 channels)
        sensors(i,18:25) = fread(fid, 8, 'float', 0, 'ieee-le');
                
        % vbat
        sensors(i,26) = fread(fid, 1, 'float', 0, 'ieee-le');
        
        % adc voltage (3 channels)
        sensors(i,27:29) = fread(fid, 3, 'float', 0, 'ieee-le');
        
        % local position (3 channels)
        sensors(i,30:32) = fread(fid, 3, 'float', 0, 'ieee-le');
        
        % gps_raw_position (3 channels)
        sensors(i,33:35) = fread(fid, 3, 'uint32', 0, 'ieee-le');
        
        % attitude (3 channels)
        sensors(i,36:38) = fread(fid, 3, 'float', 0, 'ieee-le');
        
        % RotMatrix (9 channels)
        sensors(i,39:47) = fread(fid, 9, 'float', 0, 'ieee-le');

        % vicon position (6 channels)
        sensors(i,48:53) = fread(fid, 6, 'float', 0, 'ieee-le');

        % actuator control effective (4 channels) 
        sensors(i,54:57) = fread(fid, 4, 'float', 0, 'ieee-le');

        % optical flow (6 values) 
        sensors(i,58:63) = fread(fid, 6, 'float', 0, 'ieee-le');
    end
    time_us = sensors(elements,1) - sensors(1,1);
    time_s = time_us*1e-6
    time_m = time_s/60
    disp(['end log2matlab conversion' char(10)]);
else
    disp(['file: ' filePath ' does not exist' char(10)]);
end


%% old version of reading in different files from sdlog.c
% if exist('sysvector.bin', 'file')
%     % Read actuators file
%     myFile = java.io.File('sysvector.bin')
%     fileSize = length(myFile)
% 
%     fid = fopen('sysvector.bin', 'r');
%     elements = int64(fileSize./(8+(3+3+3+1+1+1+4+8+4+3+3)*4));
% 
%     for i=1:elements
%         % timestamp
%         sysvector(i,1) = double(fread(fid, 1, '*uint64', 0, 'ieee-le.l64'));
%         % actuators 1-16
%         % quadrotor: motor 1-4 on the first four positions
%         sysvector(i, 2:32) = fread(fid, 28+3, 'float', 'ieee-le');
%         sysvector(i,33:35) = fread(fid, 3, 'int32', 'ieee-le');
%     end
%     
%     sysvector_interval_seconds = (sysvector(end,1) - sysvector(1:1)) / 1000000
%     sysvector_minutes = sysvector_interval_seconds / 60
%     
%     % Normalize time
%     sysvector(:,1) = (sysvector(:,1) - sysvector(1,1)) / 1000000;
%     
%     % Create some basic plots
%     
%     % Remove zero rows from GPS
%     gps = sysvector(:,33:35);
%     gps(~any(gps,2), :) = [];
%     
%     all_data = figure('Name', 'GPS RAW');
%     gps_position = plot3(gps(:,1), gps(:,2), gps(:,3));
%     
%     
%     all_data = figure('Name', 'Complete Log Data (exc. GPS)');
%     plot(sysvector(:,1), sysvector(:,2:32));
%     
%     actuator_inputs = figure('Name', 'Attitude controller outputs');
%     plot(sysvector(:,1), sysvector(:,14:17));
%     legend('roll motor setpoint', 'pitch motor setpoint', 'yaw motor setpoint', 'throttle motor setpoint');
%     
%     actuator_outputs = figure('Name', 'Actuator outputs');
%     plot(sysvector(:,1), sysvector(:,18:25));
%     legend('actuator 0', 'actuator 1', 'actuator 2', 'actuator 3', 'actuator 4', 'actuator 5', 'actuator 6', 'actuator 7');
%     
% end
% 
% if exist('actuator_outputs0.bin', 'file')
%     % Read actuators file
%     myFile = java.io.File('actuator_outputs0.bin')
%     fileSize = length(myFile)
% 
%     fid = fopen('actuator_outputs0.bin', 'r');
%     elements = int64(fileSize./(16*4+8))
% 
%     for i=1:elements
%         % timestamp
%         actuators(i,1) = double(fread(fid, 1, '*uint64', 0, 'ieee-le.l64'));
%         % actuators 1-16
%         % quadrotor: motor 1-4 on the first four positions
%         actuators(i, 2:17) = fread(fid, 16, 'float', 'ieee-le');
%     end
% end
% 
% if exist('actuator_controls0.bin', 'file')
%     % Read actuators file
%     myFile = java.io.File('actuator_controls0.bin')
%     fileSize = length(myFile)
% 
%     fid = fopen('actuator_controls0.bin', 'r');
%     elements = int64(fileSize./(8*4+8))
% 
%     for i=1:elements
%         % timestamp
%         actuator_controls(i,1) = fread(fid, 1, 'uint64', 0, 'ieee-le.l64');
%         % actuators 1-16
%         % quadrotor: motor 1-4 on the first four positions
%         actuator_controls(i, 2:9) = fread(fid, 8, 'float', 'ieee-le');
%     end
% end
% 
% 
% if exist('sensor_combined.bin', 'file')
%     % Read sensor combined file
%     % Type definition: Firmware/apps/uORB/topics/sensor_combined.h
%     % Struct: sensor_combined_s
%     fileInfo = dir('sensor_combined.bin');
%     fileSize = fileInfo.bytes;
% 
%     fid = fopen('sensor_combined.bin', 'r');
% 
%     for i=1:elements
%         % timestamp
%         sensors(i,1) = double(fread(fid, 1, '*uint64', 0, 'ieee-le.l64'));
%         % gyro raw
%         sensors(i,2:4) = fread(fid, 3, 'int16', 0, 'ieee-le');
%         % gyro counter
%         sensors(i,5) = fread(fid, 1, 'uint16', 0, 'ieee-le');
%         % gyro in rad/s
%         sensors(i,6:8) = fread(fid, 3, 'float', 0, 'ieee-le');
% 
%         % accelerometer raw
%         sensors(i,9:11) = fread(fid, 3, 'int16', 0, 'ieee-le');
%         % padding bytes
%         fread(fid, 1, 'int16', 0, 'ieee-le');
%         % accelerometer counter
%         sensors(i,12) = fread(fid, 1, 'uint32', 0, 'ieee-le');
%         % accel in m/s2
%         sensors(i,13:15) = fread(fid, 3, 'float', 0, 'ieee-le');
%         % accel mode
%         sensors(i,16) = fread(fid, 1, 'int32', 0, 'ieee-le');
%         % accel range
%         sensors(i,17) = fread(fid, 1, 'float', 0, 'ieee-le');
% 
%         % mag raw
%         sensors(i,18:20) = fread(fid, 3, 'int16', 0, 'ieee-le');
%         % padding bytes
%         fread(fid, 1, 'int16', 0, 'ieee-le');
%         % mag in Gauss
%         sensors(i,21:23) = fread(fid, 3, 'float', 0, 'ieee-le');
%         % mag mode
%         sensors(i,24) = fread(fid, 1, 'int32', 0, 'ieee-le');
%         % mag range
%         sensors(i,25) = fread(fid, 1, 'float', 0, 'ieee-le');
%         % mag cuttoff freq
%         sensors(i,26) = fread(fid, 1, 'float', 0, 'ieee-le');
%         % mag counter
%         sensors(i,27) = fread(fid, 1, 'int32', 0, 'ieee-le');
% 
%         % baro pressure millibar
%         % baro alt meter
%         % baro temp celcius
%         % battery voltage
%         % adc voltage (3 channels)
%         sensors(i,28:34) = fread(fid, 7, 'float', 0, 'ieee-le');
%         % baro counter and battery counter
%         sensors(i,35:36) = fread(fid, 2, 'uint32', 0, 'ieee-le');
%         % battery voltage valid flag
%         sensors(i,37) = fread(fid, 1, 'uint32', 0, 'ieee-le');
% 
%     end
% end


