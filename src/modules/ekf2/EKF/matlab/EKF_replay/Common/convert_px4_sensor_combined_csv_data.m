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
imu_data.gyro_dt = gyro_integral_dt ./ 1e6;
imu_data.del_ang = [gyro_rad0.*imu_data.gyro_dt, gyro_rad1.*imu_data.gyro_dt, gyro_rad2.*imu_data.gyro_dt];

imu_data.accel_dt = accelerometer_integral_dt ./ 1e6;
imu_data.del_vel = [accelerometer_m_s20.*imu_data.accel_dt, accelerometer_m_s21.*imu_data.accel_dt, accelerometer_m_s22.*imu_data.accel_dt];

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
