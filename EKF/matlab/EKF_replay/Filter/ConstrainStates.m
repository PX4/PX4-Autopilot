function [states]  = ConstrainStates(states,dt_imu_avg)

% constrain gyro bias states
limit = 5.0*pi/180*dt_imu_avg;
for i=11:13
    if (states(i) > limit)
        states(i) = limit;
    elseif (states(i) < -limit)
        states(i) = -limit;
    end
end

% constrain accel bias states
limit = 0.5*dt_imu_avg;
for i=14:16
    if (states(i) > limit)
        states(i) = limit;
    elseif (states(i) < -limit)
        states(i) = -limit;
    end
end

end