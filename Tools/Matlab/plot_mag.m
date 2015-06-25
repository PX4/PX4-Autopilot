%
% Tool for plotting mag data
%
% Reference values:
% telem> [cal] mag #0 off: x:0.15    y:0.07     z:0.14 Ga
%                  MATLAB: x:0.1581  y: 0.0701  z: 0.1439 Ga
% telem> [cal] mag #0 scale: x:1.10 y:0.97 z:1.02
%                  MATLAB: 0.5499, 0.5190, 0.4907
%
% telem> [cal] mag #1 off: x:-0.18    y:0.11    z:-0.09 Ga
%                  MATLAB: x:-0.1827  y:0.1147  z:-0.0848 Ga
% telem> [cal] mag #1 scale: x:1.00 y:1.00 z:1.00
%                  MATLAB: 0.5122, 0.5065, 0.4915
%
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

mag0_raw = load('../../mag0_raw2.csv');
mag1_raw = load('../../mag1_raw2.csv');

mag0_cal = load('../../mag0_cal2.csv');
mag1_cal = load('../../mag1_cal2.csv');

fm0r = figure();

mag0_x_scale = 1.07;
mag0_y_scale = 0.95;
mag0_z_scale = 1.00;

plot3(mag0_raw(:,1), mag0_raw(:,2), mag0_raw(:,3), '*r');
[center, radii, evecs, pars ] = ellipsoid_fit( [mag0_raw(:,1) mag0_raw(:,2) mag0_raw(:,3)] );
center
radii
axis([xmin xmax ymin ymax zmin zmax])

fm1r = figure();
plot3(mag1_raw(:,1), mag1_raw(:,2), mag1_raw(:,3), '*r');
[center, radii, evecs, pars ] = ellipsoid_fit( [mag1_raw(:,1) mag1_raw(:,2) mag1_raw(:,3)] );
center
radii
axis([xmin xmax ymin ymax zmin zmax])

fm0c = figure();
plot3(mag0_cal(:,1) .* mag0_x_scale, mag0_cal(:,2) .* mag0_y_scale, mag0_cal(:,3) .* mag0_z_scale, '*b');
axis([xmin xmax ymin ymax zmin zmax])

fm1c = figure();
plot3(mag1_cal(:,1), mag1_cal(:,2), mag1_cal(:,3), '*b');
axis([xmin xmax ymin ymax zmin zmax])
