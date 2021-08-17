%% convert baro data
clear baro_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_baro)
    baro_timestamp = timestamp_baro(source_index);
    if (baro_timestamp ~= last_time)
        baro_data.time_us(output_index,1) = baro_timestamp;
        baro_data.height(output_index) = baro_alt_meter(source_index);
        last_time = baro_timestamp;
        output_index = output_index + 1;
    end
end

%% convert IMU data to delta angles and velocities
% Note: these quatntis were converted from delta angles and velocites using
% the integral_dt values in the PX4 sensor module so we only need to
% multiply by integral_dt to convert back
clear imu_data;
n_samples = length(timestamp);
imu_data.time_us = timestamp + accelerometer_timestamp_relative;
imu_data.gyro_dt = delta_angle_dt ./ 1e6;
imu_data.del_ang = [delta_angle0, delta_angle1, delta_angle2];

imu_data.accel_dt = delta_velocity_dt ./ 1e6;
imu_data.del_vel = [delta_velocity0, delta_velocity1, delta_velocity2];

%% convert magnetometer data
clear mag_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_mag)
    mag_timestamp = timestamp_mag(source_index);
    if (mag_timestamp ~= last_time)
        mag_data.time_us(output_index,1) = mag_timestamp;
        mag_data.field_ga(output_index,:) = [magnetometer_ga0(source_index),magnetometer_ga1(source_index),magnetometer_ga2(source_index)];
        last_time = mag_timestamp;
        output_index = output_index + 1;
    end
end

%% save data
% DO NOT clear the workspace (yet)

save baro_data.mat baro_data;
save imu_data.mat imu_data;
save mag_data.mat mag_data;
