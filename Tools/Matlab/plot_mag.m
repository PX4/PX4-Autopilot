%
% Tool for plotting mag data
%
close all;
clear all;

plot_scale = 0.8;

xmax = plot_scale;
xmin = -xmax;
ymax = plot_scale;
ymin = -ymax;
zmax = plot_scale;
zmin = -zmax;

mag0_raw = load('../../mag0_raw.csv');
mag1_raw = load('../../mag1_raw.csv');

mag0_cal = load('../../mag0_cal.csv');
mag1_cal = load('../../mag1_cal.csv');

fm0r = figure();

mag0_x_scale = 1.07;
mag0_y_scale = 0.95;
mag0_z_scale = 1.00;

plot3(mag0_raw(:,1) .* mag0_x_scale, mag0_raw(:,2) .* mag0_y_scale, mag0_raw(:,3) .* mag0_z_scale, '*r');
axis([xmin xmax ymin ymax zmin zmax])

fm1r = figure();
plot3(mag1_raw(:,1), mag1_raw(:,2), mag1_raw(:,3), '*r');
axis([xmin xmax ymin ymax zmin zmax])

fm0c = figure();
plot3(mag0_cal(:,1), mag0_cal(:,2), mag0_cal(:,3), '*b');
axis([xmin xmax ymin ymax zmin zmax])

fm1c = figure();
plot3(mag1_cal(:,1), mag1_cal(:,2), mag1_cal(:,3), '*b');
axis([xmin xmax ymin ymax zmin zmax])
