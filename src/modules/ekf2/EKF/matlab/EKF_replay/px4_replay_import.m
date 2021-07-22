%% PX4 replay: import sensors CSV
% the following variables must be set beforehand!
if ~exist('sensors_file','var')
    error('sensors_file missing');
end
if ~exist('air_data_file','var')
    error('air_data_file missing');
end
if ~exist('magnetometer_file','var')
    error('magnetometer_file missing');
end
if ~exist('gps_file','var')
    error('gps_file missing');
end

if ~exist('attitude_file','var')
    disp('INFO no attitude_file set, all ground truth data will be skipped');
end
if ~exist('localpos_file','var')
    disp('INFO no localpos_file set, all ground truth data will be skipped');
end
if ~exist('globalpos_file','var')
    disp('INFO no globalpos_file set, all ground truth data will be skipped');
end

if ~exist('actctrl_file','var')
    disp('INFO no actctrl_file set, all actuator data will be skipped');
end
if ~exist('actout_file','var')
    disp('INFO no actout_file set, all actuator data will be skipped');
end

%% ------                 SECTION 1: IMU, Baro, Mag                 ------

%% Import IMU data from text file
opts = delimitedTextImportOptions("NumVariables", 10);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "gyro_rad0", "gyro_rad1", "gyro_rad2", "gyro_integral_dt", "accelerometer_timestamp_relative", "accelerometer_m_s20", "accelerometer_m_s21", "accelerometer_m_s22", "accelerometer_integral_dt"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(sensors_file, opts);

% Convert to output type
timestamp = tbl.timestamp;
gyro_rad0 = tbl.gyro_rad0;
gyro_rad1 = tbl.gyro_rad1;
gyro_rad2 = tbl.gyro_rad2;
gyro_integral_dt = tbl.gyro_integral_dt;
accelerometer_timestamp_relative = tbl.accelerometer_timestamp_relative;
accelerometer_m_s20 = tbl.accelerometer_m_s20;
accelerometer_m_s21 = tbl.accelerometer_m_s21;
accelerometer_m_s22 = tbl.accelerometer_m_s22;
accelerometer_integral_dt = tbl.accelerometer_integral_dt;

clear opts tbl

%% Import Baro data from text file
opts = delimitedTextImportOptions("NumVariables", 5);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp_baro", "baro_alt_meter", "baro_temp_celcius", "baro_pressure_pa", "rho"];
opts.VariableTypes = ["double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(air_data_file, opts);

% Convert to output type
timestamp_baro = tbl.timestamp_baro;
baro_alt_meter = tbl.baro_alt_meter;
baro_temp_celcius = tbl.baro_temp_celcius;
baro_pressure_pa = tbl.baro_pressure_pa;
rho = tbl.rho;

clear opts tbl

%% Import Mag data from text file
opts = delimitedTextImportOptions("NumVariables", 4);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp_mag", "magnetometer_ga0", "magnetometer_ga1", "magnetometer_ga2"];
opts.VariableTypes = ["double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(magnetometer_file, opts);

% Convert to output type
timestamp_mag = tbl.timestamp_mag;
magnetometer_ga0 = tbl.magnetometer_ga0;
magnetometer_ga1 = tbl.magnetometer_ga1;
magnetometer_ga2 = tbl.magnetometer_ga2;

clear opts tbl

%% Run conversion script for IMU, Baro and Mag
cd Common/;
convert_px4_sensor_combined_csv_data;
cd ../;


%% ------                       SECTION 2: GPS                       ------

%% Import data from text file
opts = delimitedTextImportOptions("NumVariables", 25);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "time_utc_usec", "lat", "lon", "alt", "alt_ellipsoid", "s_variance_m_s", "c_variance_rad", "eph", "epv", "hdop", "vdop", "noise_per_ms", "jamming_indicator", "vel_m_s", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "cog_rad", "timestamp_time_relative", "heading", "heading_offset", "fix_type", "vel_ned_valid", "satellites_used"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(gps_file, opts);

% Convert to output type
timestamp = tbl.timestamp;
time_utc_usec = tbl.time_utc_usec;
lat = tbl.lat;
lon = tbl.lon;
alt = tbl.alt;
alt_ellipsoid = tbl.alt_ellipsoid;
s_variance_m_s = tbl.s_variance_m_s;
c_variance_rad = tbl.c_variance_rad;
eph = tbl.eph;
epv = tbl.epv;
hdop = tbl.hdop;
vdop = tbl.vdop;
noise_per_ms = tbl.noise_per_ms;
jamming_indicator = tbl.jamming_indicator;
vel_m_s = tbl.vel_m_s;
vel_n_m_s = tbl.vel_n_m_s;
vel_e_m_s = tbl.vel_e_m_s;
vel_d_m_s = tbl.vel_d_m_s;
cog_rad = tbl.cog_rad;
timestamp_time_relative = tbl.timestamp_time_relative;
heading = tbl.heading;
heading_offset = tbl.heading_offset;
fix_type = tbl.fix_type;
vel_ned_valid = tbl.vel_ned_valid;
satellites_used = tbl.satellites_used;

clear opts tbl


%% Run conversion script for GPS
cd Common/;
convert_px4_vehicle_gps_position_csv;
cd ../;


%% ------     SECTION 3: Ground Truth Data (STIL only, optional)     ------

if exist('attitude_file','var') && exist('localpos_file','var') && exist('globalpos_file','var')
    
%- Import Attitude data from text file
opts = delimitedTextImportOptions("NumVariables", 13);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "rollspeed", "pitchspeed", "yawspeed", "q0", "q1", "q2", "q3", "delta_q_reset0", "delta_q_reset1", "delta_q_reset2", "delta_q_reset3", "quat_reset_counter"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(attitude_file, opts);

% Convert to output type
timestamp_att = tbl.timestamp;
rollspeed = tbl.rollspeed;
pitchspeed = tbl.pitchspeed;
yawspeed = tbl.yawspeed;
q0 = tbl.q0;
q1 = tbl.q1;
q2 = tbl.q2;
q3 = tbl.q3;
% delta_q_reset0 = tbl.delta_q_reset0;
% delta_q_reset1 = tbl.delta_q_reset1;
% delta_q_reset2 = tbl.delta_q_reset2;
% delta_q_reset3 = tbl.delta_q_reset3;
% quat_reset_counter = tbl.quat_reset_counter;

clear opts tbl


%- Import Global Position data from text file
opts = delimitedTextImportOptions("NumVariables", 17);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "lat", "lon", "alt", "alt_ellipsoid", "delta_alt", "vel_n", "vel_e", "vel_d", "yaw", "eph", "epv", "terrain_alt", "lat_lon_reset_counter", "alt_reset_counter", "terrain_alt_valid", "dead_reckoning"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(globalpos_file, opts);

% Convert to output type
timestamp_gpos = tbl.timestamp;
gpos_lat = tbl.lat;
gpos_lon = tbl.lon;
gpos_alt = tbl.alt;
% gpos_alt_ellipsoid = tbl.alt_ellipsoid;
% gpos_delta_alt = tbl.delta_alt;
gpos_vel_n = tbl.vel_n;
gpos_vel_e = tbl.vel_e;
gpos_vel_d = tbl.vel_d;
% gpos_yaw = tbl.yaw;
% gpos_eph = tbl.eph;
% gpos_epv = tbl.epv;
% gpos_terrain_alt = tbl.terrain_alt;
% gpos_lat_lon_reset_counter = tbl.lat_lon_reset_counter;
% gpos_alt_reset_counter = tbl.alt_reset_counter;
% gpos_terrain_alt_valid = tbl.terrain_alt_valid;
% gpos_dead_reckoning = tbl.dead_reckoning;

% Clear temporary variables
clear opts tbl


%- Import Local Position data from text file
opts = delimitedTextImportOptions("NumVariables", 43);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "ref_timestamp", "ref_lat", "ref_lon", "x", "y", "z", "delta_xy0", "delta_xy1", "delta_z", "vx", "vy", "vz", "z_deriv", "delta_vxy0", "delta_vxy1", "delta_vz", "ax", "ay", "az", "yaw", "ref_alt", "dist_bottom", "dist_bottom_rate", "eph", "epv", "evh", "evv", "vxy_max", "vz_max", "hagl_min", "hagl_max", "xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "xy_reset_counter", "z_reset_counter", "vxy_reset_counter", "vz_reset_counter", "xy_global", "z_global", "dist_bottom_valid"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(localpos_file, opts);

% Convert to output type
timestamp_lpos = tbl.timestamp;
% lpos_ref_timestamp = tbl.ref_timestamp;
lpos_ref_lat = tbl.ref_lat;
lpos_ref_lon = tbl.ref_lon;
lpos_x = tbl.x;
lpos_y = tbl.y;
lpos_z = tbl.z;
% lpos_delta_xy0 = tbl.delta_xy0;
% lpos_delta_xy1 = tbl.delta_xy1;
% lpos_delta_z = tbl.delta_z;
lpos_vx = tbl.vx;
lpos_vy = tbl.vy;
lpos_vz = tbl.vz;
% lpos_z_deriv = tbl.z_deriv;
% lpos_delta_vxy0 = tbl.delta_vxy0;
% lpos_delta_vxy1 = tbl.delta_vxy1;
% lpos_delta_vz = tbl.delta_vz;
% lpos_ax = tbl.ax;
% lpos_ay = tbl.ay;
% lpos_az = tbl.az;
lpos_yaw = tbl.yaw;
% ref_alt = tbl.ref_alt;
% dist_bottom = tbl.dist_bottom;
% dist_bottom_rate = tbl.dist_bottom_rate;
% eph = tbl.eph;
% epv = tbl.epv;
% evh = tbl.evh;
% evv = tbl.evv;
% vxy_max = tbl.vxy_max;
% vz_max = tbl.vz_max;
% hagl_min = tbl.hagl_min;
% hagl_max = tbl.hagl_max;
% xy_valid = tbl.xy_valid;
% z_valid = tbl.z_valid;
% v_xy_valid = tbl.v_xy_valid;
% v_z_valid = tbl.v_z_valid;
% xy_reset_counter = tbl.xy_reset_counter;
% z_reset_counter = tbl.z_reset_counter;
% vxy_reset_counter = tbl.vxy_reset_counter;
% vz_reset_counter = tbl.vz_reset_counter;
% xy_global = tbl.xy_global;
% z_global = tbl.z_global;
% dist_bottom_valid = tbl.dist_bottom_valid;

% Clear temporary variables
clear opts tbl


%- Run conversion script for GPS
cd Common/;
convert_px4_groundtruth_csv_data;
cd ../;

end


%% ------          SECTION 4: Actuator Controls (optional)          ------

if exist('actctrl_file','var') && exist('actout_file','var')

%- Import Actuator Control data from text file
opts = delimitedTextImportOptions("NumVariables", 10);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "timestamp_sample", "control0", "control1", "control2", "control3", "control4", "control5", "control6", "control7"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(actctrl_file, opts);

% Convert to output type
timestamp_actctrl = tbl.timestamp;
%timestamp_sample = tbl.timestamp_sample;
control0 = tbl.control0;
control1 = tbl.control1;
control2 = tbl.control2;
control3 = tbl.control3;
% control4 = tbl.control4;
% control5 = tbl.control5;
% control6 = tbl.control6;
% control7 = tbl.control7;

% Clear temporary variables
clear opts tbl


%- Import Actuator Output data from text file
opts = delimitedTextImportOptions("NumVariables", 18);
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["timestamp", "noutputs", "output0", "output1", "output2", "output3", "output4", "output5", "output6", "output7", "output8", "output9", "output10", "output11", "output12", "output13", "output14", "output15"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(actout_file, opts);

% Convert to output type
timestamp_actout = tbl.timestamp;
% noutputs = tbl.noutputs;
output0 = tbl.output0;
output1 = tbl.output1;
output2 = tbl.output2;
output3 = tbl.output3;
% output4 = tbl.output4;
% output5 = tbl.output5;
% output6 = tbl.output6;
% output7 = tbl.output7;
% output8 = tbl.output8;
% output9 = tbl.output9;
% output10 = tbl.output10;
% output11 = tbl.output11;
% output12 = tbl.output12;
% output13 = tbl.output13;
% output14 = tbl.output14;
% output15 = tbl.output15;

% Clear temporary variables
clear opts tbl


%- Run conversion script for GPS
cd Common/;
convert_px4_actuators_csv_data;
cd ../;

end
