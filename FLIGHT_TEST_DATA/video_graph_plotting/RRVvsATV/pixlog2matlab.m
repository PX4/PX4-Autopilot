clear all;
close all;

%% README
% this script plots data for attitude estimation of two vehicles connected
%   a QGroundControl station. This is a offline script that reads from a
%   postflight or postexperiment log file *.log from QGC.
% this script assumes two vehicles ATV and RRV are connected to the
%   QGroundStation, with IDs 1 (RRV) and 4 (ATV). Additionally, we assume
%   that the only messages that are being sent are messages of roll, pitch
%   and yaw for both vehicles. Additional messages in the log will preclude
%   this script from working correctly. This should be addressed in the
%   future.

%% opens log file from QGroundControl station
fileID = fopen('logs/may22-2019.log');

%% this is the file format for each line
formatSpec = '%f %f %s %f';

%% scan all lines using the format above
A = textscan(fileID,formatSpec);

time      = A{1};
vehicleID = A{2};
valueName = A{3};
value     = A{4};

%% collect different data in separate vectors
roll = [];
chan4 = [];
chan5 = [];
chan6 = [];
servo = [];
for i = 1:length(time)
    if ( strcmp(valueName(i), 'M4:ATTITUDE.roll') ) 
        roll = [roll; time(i) value(i) ];
    end
    if ( strcmp(valueName(i), 'M4:port0_RC_CHANNELS_SCALED.chan4_scaled') ) 
        chan4 = [chan4; time(i) value(i) ];
    end
    if ( strcmp(valueName(i), 'M4:port0_RC_CHANNELS_SCALED.chan5_scaled') ) 
        chan5 = [chan5; time(i) value(i) ];
    end
    if ( strcmp(valueName(i), 'M4:port0_RC_CHANNELS_SCALED.chan6_scaled') ) 
        chan6 = [chan6; time(i) value(i) ];
    end
    if ( strcmp(valueName(i), 'M4:port0_SERVO_OUTPUT_RAW.servo10_raw') ) 
        servo = [servo; time(i) value(i) ];
    end
end

%% plot results
figure;
subplot(4,1,1);
plot(chan5(:,1), chan5(:,2), chan6(:,1), chan6(:,2));
title('Flight mode switches');
xlabel('Time (s)');
ylabel('Flight Modes');
ylim([-1000 11000]);
legend('SYSID SW', 'STAB SW');

subplot(4,1,2);
plot(chan4(:,1), 1/100*chan4(:,2));
title('Pilot roll command');
xlabel('Time (s)');
ylabel('RC (%)');
%xlim([0 43]);

subplot(4,1,3);
plot(roll(:,1), 180/pi*roll(:,2));
title('Roll angle');
xlabel('Time (s)');
ylabel('Roll (deg)');
%xlim([0 43]);

subplot(4,1,4);
plot(servo(:,1), servo(:,2));
title('Aileron spoiler');
xlabel('Time (s)');
ylabel('Servo wave (us)');
%xlim([0 43]);
