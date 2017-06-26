%% convert baro data
clear flow_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
    flow_timestamp = timestamp(source_index);
    if ((flow_timestamp ~= last_time) && (integration_timespan(source_index) > 0))
        last_time = flow_timestamp;
        flow_data.time_us(output_index,1) = flow_timestamp;
        flow_data.qual(output_index,1) = quality(source_index)/255; % scale quality from 0 to 1
        dt_inv = 1e6 / integration_timespan(source_index);
        flow_data.flowX(output_index,1) = dt_inv * pixel_flow_x_integral(source_index); % optical flow rate about the X body axis (rad/sec)
        flow_data.flowY(output_index,1) = dt_inv * pixel_flow_y_integral(source_index); % optical flow rate about the Y body axis (rad/sec)
        flow_data.bodyX(output_index,1) = dt_inv * gyro_x_rate_integral(source_index); % angular rate about the X body axis (rad/sec)
        flow_data.bodyY(output_index,1) = dt_inv * gyro_y_rate_integral(source_index); % angular rate about the Y body axis (rad/sec)
        output_index = output_index + 1;
    end
end

%% save data and clear workspace
clearvars -except flow_data;

save flow_data.mat flow_data;