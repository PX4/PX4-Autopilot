%% convert attitude data
clear attitude_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_att)
    att_timestamp = timestamp_att(source_index);
    if (att_timestamp ~= last_time)
        attitude_data.time_us(output_index,1) = att_timestamp;
        attitude_data.quat(output_index,:) = [q0(source_index),q1(source_index),q2(source_index),q3(source_index)];
        attitude_data.del_ang(output_index,:) = [rollspeed(source_index),pitchspeed(source_index),yawspeed(source_index)];
        last_time = att_timestamp;
        output_index = output_index + 1;
    end
end

%% convert global position data
clear globalpos_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_gpos)
    gpos_timestamp = timestamp_gpos(source_index);
    if (gpos_timestamp ~= last_time)
        globalpos_data.time_us(output_index,1) = gpos_timestamp;
        globalpos_data.position_NED(output_index,:) = [gpos_lat(source_index),gpos_lon(source_index),gpos_alt(source_index)];
        globalpos_data.velocity_NED(output_index,:) = [gpos_vel_n(source_index),gpos_vel_e(source_index),gpos_vel_d(source_index)];
        last_time = gpos_timestamp;
        output_index = output_index + 1;
    end
end

%% convert local position data
clear localpos_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_lpos)
    lpos_timestamp = timestamp_lpos(source_index);
    if (lpos_timestamp ~= last_time)
        localpos_data.time_us(output_index,1) = lpos_timestamp;
        localpos_data.ref_pos(output_index,:) = [lpos_ref_lat(source_index),lpos_ref_lon(source_index)];
        localpos_data.position_XYZ(output_index,:) = [lpos_x(source_index),lpos_y(source_index),lpos_z(source_index)];
        localpos_data.velocity_XYZ(output_index,:) = [lpos_vz(source_index),lpos_vy(source_index),lpos_vz(source_index)];
        localpos_data.yaw(output_index,1) = lpos_yaw(source_index);
        last_time = lpos_timestamp;
        output_index = output_index + 1;
    end
end

%% save data
% DO NOT clear the workspace (yet)

save attitude_data.mat attitude_data;
save localpos_data.mat localpos_data;
save globalpos_data.mat globalpos_data;
