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
% User-guided values:
%
% telem> [cal] mag #0 off: x:0.12 y:0.09 z:0.14 Ga
% telem> [cal] mag #0 scale: x:0.88 y:0.99 z:0.95
% telem> [cal] mag #1 off: x:-0.18 y:0.11 z:-0.09 Ga
% telem> [cal] mag #1 scale: x:1.00 y:1.00 z:1.00

close all;
clear all;

plot_scale = 0.8;

xmax = plot_scale;
xmin = -xmax;
ymax = plot_scale;
ymin = -ymax;
zmax = plot_scale;
zmin = -zmax;

mag0_raw = load('../../mag0_raw3.csv');
mag1_raw = load('../../mag1_raw3.csv');

mag0_cal = load('../../mag0_cal3.csv');
mag1_cal = load('../../mag1_cal3.csv');

fm0r = figure();

mag0_x_scale = 0.88;
mag0_y_scale = 0.99;
mag0_z_scale = 0.95;

plot3(mag0_raw(:,1), mag0_raw(:,2), mag0_raw(:,3), '*r');
[mag0_raw_center, mag0_raw_radii, evecs, pars ] = ellipsoid_fit( [mag0_raw(:,1) mag0_raw(:,2) mag0_raw(:,3)] );
mag0_raw_center
mag0_raw_radii
axis([xmin xmax ymin ymax zmin zmax])
viscircles([mag0_raw_center(1), mag0_raw_center(2)], [mag0_raw_radii(1)]);

fm1r = figure();
plot3(mag1_raw(:,1), mag1_raw(:,2), mag1_raw(:,3), '*r');
[center, radii, evecs, pars ] = ellipsoid_fit( [mag1_raw(:,1) mag1_raw(:,2) mag1_raw(:,3)] );
center
radii
axis([xmin xmax ymin ymax zmin zmax])

fm0c = figure();
plot3(mag0_cal(:,1) .* mag0_x_scale, mag0_cal(:,2) .* mag0_y_scale, mag0_cal(:,3) .* mag0_z_scale, '*b');
[mag0_cal_center, mag0_cal_radii, evecs, pars ] = ellipsoid_fit( [mag1_raw(:,1) .* mag0_x_scale mag1_raw(:,2) .* mag0_y_scale mag1_raw(:,3) .* mag0_z_scale] );
mag0_cal_center
mag0_cal_radii
axis([xmin xmax ymin ymax zmin zmax])
viscircles([0, 0], [mag0_cal_radii(3)]);

fm1c = figure();
plot3(mag1_cal(:,1), mag1_cal(:,2), mag1_cal(:,3), '*b');
axis([xmin xmax ymin ymax zmin zmax])
[center, radii, evecs, pars ] = ellipsoid_fit( [mag1_raw(:,1) mag1_raw(:,2) mag1_raw(:,3)] );
viscircles([0, 0], [radii(3)]);

mag0_x_scale_matlab = 1 / (mag0_cal_radii(1) / mag0_raw_radii(1))
mag0_y_scale_matlab = 1 / (mag0_cal_radii(2) / mag0_raw_radii(2))
mag0_z_scale_matlab = 1 / (mag0_cal_radii(3) / mag0_raw_radii(3))
