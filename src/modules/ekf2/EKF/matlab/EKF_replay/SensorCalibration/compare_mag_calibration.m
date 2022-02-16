% test ellipsoid sphere fitting algorithms
%
% http://www.st.com/content/ccc/resource/technical/document/design_tip/group0/a2/98/f5/d4/9c/48/4a/d1/DM00286302/files/DM00286302.pdf/jcr:content/translations/en.DM00286302.pdf
%

%% load log data
clear all;
close all;

% uncomment these lines if using legacy .px4log format
%load sysvector.mat;
%mag_meas = [sysvector.IMU_MagX';sysvector.IMU_MagY';sysvector.IMU_MagZ'];

% uncomment these lines if using data imported from ulog format
%load sysdata.mat;
%mag_meas = [magnetometer_ga0';magnetometer_ga1';magnetometer_ga2'];

% thin data points to use data every 5 deg
delta_angle_lim = 5* pi/180;
counter = 1;

angle = 0;
last_angle = 0;
for i = 2:length(data1.IMU.Tsec)
    ang_rate = 0.5 * (sqrt(data1.IMU.GyroX(i)^2 + data1.IMU.GyroY(i)^2 + data1.IMU.GyroZ(i)^2) + ...
        sqrt(data1.IMU.GyroX(i-1)^2 + data1.IMU.GyroY(i-1)^2 + data1.IMU.GyroZ(i-1)^2));
    dt = data1.IMU.Tsec(i) - data1.IMU.Tsec(i-1);
    angle = angle + ang_rate * dt;
    if ((angle - last_angle) > delta_angle_lim)
        mag_meas(:,counter) = [data1.IMU.MagX(i);data1.IMU.MagY(i);data1.IMU.MagZ(i)];
        counter = counter + 1;
        last_angle = angle;
    end
end

angle = 0;
last_angle = 0;
for i = 2:length(data2.IMU.Tsec)
    ang_rate = 0.5 * (sqrt(data2.IMU.GyroX(i)^2 + data2.IMU.GyroY(i)^2 + data2.IMU.GyroZ(i)^2) + ...
        sqrt(data2.IMU.GyroX(i-1)^2 + data2.IMU.GyroY(i-1)^2 + data2.IMU.GyroZ(i-1)^2));
    dt = data2.IMU.Tsec(i) - data2.IMU.Tsec(i-1);
    angle = angle + ang_rate * dt;
    if ((angle - last_angle) > delta_angle_lim)
        mag_meas(:,counter) = [data2.IMU.MagX(i);data2.IMU.MagY(i);data2.IMU.MagZ(i)];
        counter = counter + 1;
        last_angle = angle;
    end

end

angle = 0;
last_angle = 0;
for i = 2:length(data3.IMU.Tsec)
    ang_rate = 0.5 * (sqrt(data3.IMU.GyroX(i)^2 + data3.IMU.GyroY(i)^2 + data3.IMU.GyroZ(i)^2) + ...
        sqrt(data3.IMU.GyroX(i-1)^2 + data3.IMU.GyroY(i-1)^2 + data3.IMU.GyroZ(i-1)^2));
    dt = data3.IMU.Tsec(i) - data3.IMU.Tsec(i-1);
    angle = angle + ang_rate * dt;
    if ((angle - last_angle) > delta_angle_lim)
        mag_meas(:,counter) = [data3.IMU.MagX(i);data3.IMU.MagY(i);data3.IMU.MagZ(i)];
        counter = counter + 1;
        last_angle = angle;
    end

end


%% fit a sphere and determine the fit quality
[offset,gain,rotation]=ellipsoid_fit(mag_meas',5);

% correct the data
mag_corrected_5 = zeros(size(mag_meas));
rotation_correction = inv(rotation); % we apply the inverse of the original rotation
scale_correction = 1./gain;
scale_correction = scale_correction ./ mean(scale_correction);
mag_strength = zeros(length(mag_meas),1);
for i = 1:length(mag_meas)
    % subtract the offsets
    mag_corrected_5(:,i) = mag_meas(:,i) - offset;

    % correct the rotation
    mag_corrected_5(:,i) = rotation_correction * mag_corrected_5(:,i);

    % correct the scale factor
    mag_corrected_5(:,i) = mag_corrected_5(:,i) .* scale_correction;

    % calculate the residual
    mag_strength(i) = sqrt(dot(mag_corrected_5(:,i),mag_corrected_5(:,i)));

end

% calculate the fit residual for fit option 5
fit_residual_5 = mag_strength - mean(mag_strength);

%% fit a un-rotated ellipsoid and determine the fit quality
[offset,gain,rotation]=ellipsoid_fit(mag_meas',1);

% correct the data
mag_corrected_1 = zeros(size(mag_meas));
rotation_correction = inv(rotation); % we apply the inverse of the original rotation
scale_correction = 1./gain;
scale_correction = scale_correction ./ mean(scale_correction);
mag_strength = zeros(length(mag_meas),1);
angle_change_1 = zeros(length(mag_meas),1);
for i = 1:length(mag_meas)
    % subtract the offsets
    mag_corrected_1(:,i) = mag_meas(:,i) - offset;

    % correct the rotation
    mag_corrected_1(:,i) = rotation_correction * mag_corrected_1(:,i);

    % correct the scale factor
    mag_corrected_1(:,i) = mag_corrected_1(:,i) .* scale_correction;

    % calculate the residual
    mag_strength(i) = sqrt(dot(mag_corrected_1(:,i),mag_corrected_1(:,i)));

    % calculate the angular change due to the fit
    angle_change_1(i) = atan2(norm(cross(mag_corrected_1(:,i),mag_meas(:,i))),dot(mag_corrected_1(:,i),mag_meas(:,i)));

end

% calculate the fit residual for fit option 1
fit_residual_1 = mag_strength - mean(mag_strength);

%% fit a rotated ellipsoid and check the fit quality
[offset,gain,rotation]=ellipsoid_fit(mag_meas',0);

% correct the data
mag_corrected_0 = zeros(size(mag_meas));
rotation_correction = inv(rotation); % we apply the inverse of the original rotation
scale_correction = 1./gain;
scale_correction = scale_correction ./ mean(scale_correction);
mag_strength = zeros(length(mag_meas),1);
angle_change_0 = zeros(length(mag_meas),1);
for i = 1:length(mag_meas)
    % subtract the offsets
    mag_corrected_0(:,i) = mag_meas(:,i) - offset;

    % correct the rotation
    mag_corrected_0(:,i) = rotation_correction * mag_corrected_0(:,i);

    % correct the scale factor
    mag_corrected_0(:,i) = mag_corrected_0(:,i) .* scale_correction;

    % calculate the residual
    mag_strength(i) = sqrt(dot(mag_corrected_0(:,i),mag_corrected_0(:,i)));

    % calculate the angular change due to the fit
    angle_change_0(i) = atan2(norm(cross(mag_corrected_0(:,i),mag_meas(:,i))),dot(mag_corrected_0(:,i),mag_meas(:,i)));

end

% calculate the fit residual for fit option 0
fit_residual_0 = mag_strength - mean(mag_strength);

%% calculate the residual for uncorrected data
for i = 1:length(mag_meas)
    mag_strength(i) = sqrt(dot(mag_meas(:,i),mag_meas(:,i)));
end
uncorrected_residual = mag_strength - mean(mag_strength);

%% plot the fit residuals
plot(uncorrected_residual,'k+');
hold on;
plot(fit_residual_5,'r+');
plot(fit_residual_1,'b+');
plot(fit_residual_0,'g+');
hold off;
grid on;
title('mag fit comparison');
xlabel('measurement index');
ylabel('fit residual (Gauss)');
legend('uncorrected','sphere','non-rotated ellipse','rotated ellipse');

%% plot the data points in 3D
figure;
plot3(mag_meas(1,:),mag_meas(2,:),mag_meas(3,:),' .');hold on;plot3(mag_corrected_1(1,:),mag_corrected_1(2,:),mag_corrected_1(3,:),'r.');
hold off;grid on;axis equal;
xlabel('x (Gauss)');
xlabel('y (Gauss)');
xlabel('z (Gauss)');
legend('uncorrected','unrotated ellipse');

%% calculate and plot the angular error
figure;
plot(angle_change_1*(180/pi),'b+');
title('angle change after un-rotated ellipse fit');
xlabel('measurement index');
ylabel('angle change magnitude (deg)');
