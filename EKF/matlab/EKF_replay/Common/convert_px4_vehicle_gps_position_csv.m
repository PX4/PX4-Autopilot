%% convert GPS data
clear gps_data;
gps_data.time_us = timestamp + timestamp_time_relative;
gps_data.pos_error = eph;
gps_data.spd_error = s_variance_m_s;
gps_data.hgt_error = epv;

% set reference point used to set NED origin when GPS accuracy is sufficient
gps_data.start_index = max(min(find(gps_data.pos_error < 5.0)),min(find(gps_data.spd_error < 1.0)));
if isempty(gps_data.start_index)
    gps_data.start_index = 1;
    gps_data.refLLH = [1e-7*median(lat);1e-7*median(lon);0.001*median(alt)];
else
    gps_data.refLLH = [1e-7*lat(gps_data.start_index);1e-7*lon(gps_data.start_index);0.001*alt(gps_data.start_index)];
end

% convert GPS data to NED
for index = 1:length(timestamp)
    if (fix_type(index) >= 3)
        gps_data.pos_ned(index,:) = LLH2NED([1e-7*lat(index);1e-7*lon(index);0.001*alt(index)],gps_data.refLLH);
        gps_data.vel_ned(index,:) = [vel_n_m_s(index),vel_e_m_s(index),vel_d_m_s(index)];
    else
        gps_data.pos_ned(index,:) = [0,0,0];
        gps_data.vel_ned(index,:) = [0,0,0];
    end
end


%% save data
% DO NOT clear the workspace (yet)

save gps_data.mat gps_data;
