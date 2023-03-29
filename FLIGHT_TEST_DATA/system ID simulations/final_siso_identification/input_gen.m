% Define the time vector
t_final = 3; % Total duration of the signal
sample_rate = 50;
dt = 1/sample_rate; % Time step
time = 0:dt:t_final; % Time vector
tinit = 1;   %doublet initialized at this time

% Define the magnitude and duration
doublet_magnitude = 0.1; % Magnitude of the doublet input
doublet_duration = 0.5; % Duration of the doublet input
sine_magnitude = 0.1; % Magnitude of the sine sweep input
f_start = 0.1; % Starting frequency
f_end = 10; % Ending frequency

% Generate the doublet input
doublet = zeros(1, length(time)); % initialize output signal

for i=1:length(time)
    if (time(i)>tinit & time(i)<tinit+doublet_duration)
        if (time(i)<tinit+doublet_duration/2)
            doublet(i) = doublet_magnitude;
        elseif (time(i)==tinit+doublet_duration/2)
            doublet(i) = 0;
        else
            doublet(i) = -doublet_magnitude;
        end
    end
end

% Generate the sine sweep input
sine_sweep = zeros(1, length(time)); % initialize output signal
for i=1:length(time)
    if (time(i)>tinit)
        sine_sweep(i) = sine_magnitude * chirp(time(i),f_start,t_final,f_end);
    end
end

% Plot the inputs
figure;
subplot(2,1,1);
plot(time,doublet);
xlabel('Time (s)');
ylabel('Doublet Input');

subplot(2,1,2);
plot(time,sine_sweep);
xlabel('Time (s)');
ylabel('Sine Sweep Input');

% Save the inputs as .mat files
% save('doublet_phugoid_t.mat','time','doublet');
% save('sine_sweep_phugoid_t.mat','time','sine_sweep');
% save('time_phugoid.mat','time','time');

save('doublet_short.mat','time','doublet');
save('sine_sweep_short.mat','time','sine_sweep');
save('time_short.mat','time','time');
