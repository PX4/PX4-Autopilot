lfile = 'log001.px4log';
dfile = 'out.csv';

% check if datafile already exist 
if not(exist(dfile, 'file'))
    disp(['datafile does not exist: generating from log..' char(10)]);
    %read time, IMU (raw) and IMUT (temp compensated) data
    s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -t "TIME" -d "," -n "" -m TIME -m IMU -m IMUT', lfile, dfile) );
end

if exist(dfile, 'file')
    disp(['reading datafile..' char(10)]);
    sysvector = tdfread(dfile, ',');

    % shot the flight time
    time_us = sysvector.TIME_StartTime(end) - sysvector.TIME_StartTime(1);
    time_s = uint64(time_us*1e-6);
    time_m = uint64(time_s/60);
    time_s = time_s - time_m * 60;
        
    disp([sprintf('Flight log duration: %d:%d (minutes:seconds)', time_m, time_s) char(10)]);

    disp(['logfile conversion finished.' char(10)]);
else
    disp(['file: ' dfile ' does not exist' char(10)]);
end

% timestamps
timestamps  = sysvector.TIME_StartTime - sysvector.TIME_StartTime(1);
% raw data
accel       = [sysvector.IMU_AccX , sysvector.IMU_AccY , sysvector.IMU_AccZ ];
gyro        = [sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ];
mag         = [sysvector.IMU_MagX , sysvector.IMU_MagY , sysvector.IMU_MagZ ];
% temperatures
accel_t     = sysvector.IMU_tA;
gyro_t      = sysvector.IMU_tG;
mag_t       = sysvector.IMU_tM;
% compensated data
accel_tc    = [sysvector.IMUT_AccX , sysvector.IMUT_AccY , sysvector.IMUT_AccZ ];
gyro_tc     = [sysvector.IMUT_GyroX, sysvector.IMUT_GyroY, sysvector.IMUT_GyroZ];
mag_tc      = [sysvector.IMUT_MagX , sysvector.IMUT_MagY , sysvector.IMUT_MagZ ];


%%%%%%%%%%%%
%%% COMPENSATION

%result data
accel_comp = zeros(size(accel));
gyro_comp  = zeros(size(gyro));
mag_comp   = zeros(size(mag));

% 'true' (unbiased) temperature
ttemp = 25;

% calculate polynomials for gyro
% IN
% - gyro/accel/mag data
% - 4th order polynomial for attitude compensation (3 after differentiating)
% - imu temperature
% - 3th order polynomial for temperature compensation
% - timestamps
% - 25 degrees celcius, reference temperature of 'correct' (unbiased by temperature) measurement
% OUT
% - gyro_pt, temperature polynomials (4x3 matrix)
% - gyro_pd, attitude polynomials (4x3 matrix)
[accel_pt, accel_pd] = calcpoly(accel, 4, accel_t, 3, timestamps, ttemp);
[ gyro_pt,  gyro_pd] = calcpoly(gyro , 4, gyro_t , 3, timestamps, ttemp);
[  mag_pt,   mag_pd] = calcpoly(mag  , 4, mag_t  , 3, timestamps, ttemp);

%data size
dsize = size(gyro, 1);
ddim  = size(gyro, 2);

%simple calculations (no vectorisation) to show solution
for i=1:dsize
    a_temp = accel_t(i);
    g_temp = gyro_t(i);
    m_temp = mag_t(i);
    time = timestamps(i);

    for j=1:ddim
        accel_comp(i,j) = accel(i,j);
        gyro_comp(i,j)  =  gyro(i,j);
        mag_comp(i,j)   =   mag(i,j);

        %temperature correction
        accel_comp(i,j) = accel_comp(i,j) - (accel_pt(1,j)*a_temp^3 + accel_pt(2,j)*a_temp^2 + accel_pt(3,j)*a_temp^1);
        gyro_comp(i,j)  = gyro_comp(i,j)  - ( gyro_pt(1,j)*g_temp^3 +  gyro_pt(2,j)*g_temp^2 +  gyro_pt(3,j)*g_temp^1);
        mag_comp(i,j)   = mag_comp(i,j)   - (  mag_pt(1,j)*m_temp^3 +   mag_pt(2,j)*m_temp^2 +   mag_pt(3,j)*m_temp^1);

        %calculate offset of the 'true temperature' (result of fit should be 0);
        accel_comp(i,j) = accel_comp(i,j) + (accel_pt(1,j)*ttemp^3 + accel_pt(2,j)*ttemp^2 + accel_pt(3,j)*ttemp^1);
        gyro_comp(i,j)  = gyro_comp(i,j)  + ( gyro_pt(1,j)*ttemp^3 +  gyro_pt(2,j)*ttemp^2 +  gyro_pt(3,j)*ttemp^1);
        mag_comp(i,j)   = mag_comp(i,j)   + (  mag_pt(1,j)*ttemp^3 +   mag_pt(2,j)*ttemp^2 +   mag_pt(3,j)*ttemp^1);
    end
end

% Integrate the gyros for validation

attitude = [0 0 0];
attitude_comp = [0 0 0];

dtimes = diff(timestamps);
for i=2:size(gyro,1)
attitude(i, :)      = attitude(i - 1, :)      + gyro(i, :)      * dtimes(i-1);
attitude_comp(i, :) = attitude_comp(i - 1, :) + gyro_comp(i, :) * dtimes(i-1);
end

% PLOT MEASUERMENTS
fig = figure;
set(fig, 'Name', '1600s log');
set(fig, 'Position', [100, 100, 1049, 895]);

subplot(3,3,1);
plot(timestamps, accel);
title('Accel ({m/s^2}) - UNCOMP',...
      'FontWeight','bold')

subplot(3,3,2);
plot(timestamps, gyro);
title('Gyro (rad/s) - UNCOMP',...
      'FontWeight','bold')

subplot(3,3,3);
plot(timestamps, mag);
title('Mag - UNCOMP',...
      'FontWeight','bold')

subplot(3,3,4);
plot(timestamps, accel_comp);
title('Accel ({m/s^2}) - COMP',...
      'FontWeight','bold')

subplot(3,3,5);
plot(timestamps, gyro_comp);
title('Gyro (rad/s) - COMP',...
      'FontWeight','bold')

subplot(3,3,6);
plot(timestamps, mag_comp);
title('Mag - COMP',...
      'FontWeight','bold')

subplot(3,3,7);
plot(timestamps, accel_t);
title('Temp (^{o}C)',...
      'FontWeight','bold')

subplot(3,3,8);
plot(timestamps, gyro_t);
title('Temp (^{o}C)',...
      'FontWeight','bold')

subplot(3,3,9);
plot(timestamps, mag_t);
title('Temp (^{o}C)',...
      'FontWeight','bold')

for i=1:9
subplot(3,3,i);
xlim([0,max(timestamps)]);
yt = get(gca,'YTick');
set(gca,'YTickLabel', sprintf('%.3f|',yt));
end

fig = figure;
set(fig, 'Name', 'Gyro vs temperature');
set(fig, 'Position', [100, 100, 1049, 895]);

subplot(2,1,1);
plot(gyro_t, gyro);
title('UNCOMP gyro',...
      'FontWeight','bold')

subplot(2,1,2);
plot(gyro_t, gyro_tc);
title('onboard COMP gyro',...
      'FontWeight','bold')

fig = figure;
set(fig, 'Name', 'Gyro (rad/s) vs time');
set(fig, 'Position', [100, 100, 1049, 895]);

subplot(3,1,1);
plot(timestamps, gyro);
title('UNCOMP gyro',...
      'FontWeight','bold')

subplot(3,1,2);
plot(timestamps, gyro_comp);
title('Matlab COMP gyro',...
      'FontWeight','bold')

subplot(3,1,3);
plot(timestamps, gyro_tc);
title('onboard COMP gyro',...
      'FontWeight','bold')



fprintf('/***** ACCEL *****/ \n');
fprintf('accel_scale.cal_temp = %0.2f;\n', ttemp);
fprintf('accel_scale.min_temp = %0.2f;\n', min(gyro_t));
fprintf('accel_scale.max_temp = %0.2f;\n', max(gyro_t));

for i=1:3
    for j=1:3
        fprintf('accel_scale.x%d_temp[%d] = %0.25f;\n', 4-j, i-1, single(accel_pt(j,i)));
    end
end

fprintf('/***** GYRO *****/ \n');
fprintf('gyro_scale[s].cal_temp = %0.2f;\n', ttemp);
fprintf('gyro_scale[s].min_temp = %0.2f;\n', min(gyro_t));
fprintf('gyro_scale[s].max_temp = %0.2f;\n', max(gyro_t));

for i=1:3
    for j=1:3
        fprintf('gyro_scale[s].x%d_temp[%d] = %0.25f;\n', 4-j, i-1, single(gyro_pt(j,i)));
    end
end

fprintf('/***** MAG *****/ \n');
fprintf('mscale.cal_temp = %0.2f;\n', ttemp);
fprintf('mscale.min_temp = %0.2f;\n', min(gyro_t));
fprintf('mscale.max_temp = %0.2f;\n', max(gyro_t));

for i=1:3
    for j=1:3
        fprintf('mscale.x%d_temp[%d] = %0.25f;\n', 4-j, i-1, single(accel_pt(j,i)));
    end
end
