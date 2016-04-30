% script to test complementary output filter gain structure
vel_state = 0.0;
pos_state = 1.0;
vel_demand = 0.0;
pos_demand = 0.0;
dt = 0.01;
buffer_delay = 30;
vel_state_history = zeros(1,10000);
pos_state_history = zeros(1,10000);
time = [0:dt:9999*dt];
vel_correction = 0;
pos_correction = 0;
omega = 1.0/(dt*buffer_delay);
prev_vel = 1.0;
for i=1:10000
    prev_vel = vel_state;
    vel_state = vel_state + vel_correction*dt;
    vel_state_history(i) = vel_state;
    pos_state = pos_state + 0.5*dt*(vel_state + prev_vel) + pos_correction*dt;
    pos_state_history(i) = pos_state;
    if i > buffer_delay
        vel_correction = (vel_demand - vel_state_history(i-buffer_delay))*omega*0.5;
        pos_correction = (pos_demand - pos_state_history(i-buffer_delay))*omega*0.6;
    else
        vel_correction = 0;
        pos_correction = 0;
    end
end
min(vel_state_history)
close all;
figure;
subplot(2,1,1)
plot(time,vel_state_history);
subplot(2,1,2);
plot(time,pos_state_history);