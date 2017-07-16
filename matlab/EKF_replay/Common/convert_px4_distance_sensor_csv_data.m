%% convert baro data
clear rng_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
    rng_timestamp = timestamp(source_index);
    if (rng_timestamp ~= last_time)
        last_time = rng_timestamp;
        rng_data.time_us(output_index,1) = rng_timestamp;
        rng_data.dist(output_index,1) = current_distance(source_index);
        output_index = output_index + 1;
    end
end

%% save data and clear workspace
clearvars -except rng_data;

save rng_data.mat rng_data;