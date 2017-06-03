%% convert baro data
clear baro_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
    baro_timestamp = timestamp(source_index) + baro_timestamp_relative(source_index);
    if (baro_timestamp ~= last_time)
        baro_data.time_us(output_index,1) = baro_timestamp;
        baro_data.height(output_index) = baro_alt_meter(source_index);
        last_time = baro_timestamp;
        output_index = output_index + 1;
    end
end

%% convert IMU data to delta angles and velocities using trapezoidal integration
clear imu_data;
n_samples = length(timestamp);
imu_data.time_us = timestamp(2:n_samples) + accelerometer_timestamp_relative(2:n_samples);
imu_data.gyro_dt = gyro_integral_dt(2:n_samples);
imu_data.del_ang = 0.5 * ([gyro_rad0(1:n_samples-1).*imu_data.gyro_dt, ...
    gyro_rad1(1:n_samples-1).*imu_data.gyro_dt, ...
    gyro_rad2(1:n_samples-1).*imu_data.gyro_dt] + ...
    [gyro_rad0(2:n_samples).*imu_data.gyro_dt, ...
    gyro_rad1(2:n_samples).*imu_data.gyro_dt, ...
    gyro_rad2(2:n_samples).*imu_data.gyro_dt]);

imu_data.accel_dt = accelerometer_integral_dt(2:n_samples);
imu_data.del_vel = 0.5 * ([accelerometer_m_s20(1:n_samples-1).*imu_data.accel_dt, ...
    accelerometer_m_s21(1:n_samples-1).*imu_data.accel_dt, ...
    accelerometer_m_s22(1:n_samples-1).*imu_data.accel_dt] + ...
    [accelerometer_m_s20(2:n_samples).*imu_data.accel_dt, ...
    accelerometer_m_s21(2:n_samples).*imu_data.accel_dt, ...
    accelerometer_m_s22(2:n_samples).*imu_data.accel_dt]);

%% convert magnetomer data
clear mag_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
    mag_timestamp = timestamp(source_index) + magnetometer_timestamp_relative(source_index);
    if (mag_timestamp ~= last_time)
        mag_data.time_us(output_index,1) = mag_timestamp;
        mag_data.field_ga(output_index,:) = [magnetometer_ga0(source_index),magnetometer_ga1(source_index),magnetometer_ga2(source_index)];
        last_time = mag_timestamp;
        output_index = output_index + 1;
    end
end

%% save data and clear workspace
clearvars -except baro_data imu_data mag_data gps_data;

save baro_data.mat baro_data;
save imu_data.mat imu_data;
save mag_data.mat mag_data;
