%% convert actuator control data
clear actctrl_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_actctrl)
    actctrl_timestamp = timestamp_actctrl(source_index);
    if (actctrl_timestamp ~= last_time)
        actctrl_data.time_us(output_index,1) = actctrl_timestamp;
        actctrl_data.M_XYZ(output_index,:) = [control0(source_index),control1(source_index),control2(source_index)];
        actctrl_data.F_t(output_index,:) = control3(source_index);
        last_time = actctrl_timestamp;
        output_index = output_index + 1;
    end
end

%% convert actuator output data
clear actout_data;
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp_actout)
    actout_timestamp = timestamp_actout(source_index);
    if (actout_timestamp ~= last_time)
        actout_data.time_us(output_index,1) = actout_timestamp;
        actout_data.pwm(output_index,:) = [output0(source_index),output1(source_index),output2(source_index),output3(source_index)];
        last_time = actout_timestamp;
        output_index = output_index + 1;
    end
end

%% save data
% DO NOT clear the workspace (yet)

save actctrl_data.mat actctrl_data;
save actout_data.mat actout_data;
