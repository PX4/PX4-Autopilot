% This script generates c code required to calculate the variance of the
% TAS, AoA and AoS estimates calculated from the vehicle quaternions, NED
% velocity and NED wind velocity. Uncertainty in the quaternions is
% ignored. Variance in vehicle velocity and wind velocity is accounted for.

%% calculate TAS error terms
clear all;
reset(symengine);

syms vn ve vd 'real' % navigation frame NED velocity (m/s)
syms vwn vwe vwd 'real' % navigation frame NED wind velocity (m/s)
syms vn_var ve_var vd_var 'real' % navigation frame NED velocity variances (m/s)^2
syms vwn_var vwe_var vwd_var 'real' % navigation frame NED wind velocity variances (m/s)^2
syms q0 q1 q2 q3 'real' % quaternions defining rotation from navigation NED frame to body XYZ frame

quat = [q0;q1;q2;q3];

% rotation matrix from navigation to body frame
Tnb = transpose(Quat2Tbn(quat));

% crete velocity vectors
ground_velocity_truth = [vn;ve;vd];
wind_velocity_truth = [vwn;vwe;vwd];

% calcuate wind relative velocity
rel_vel_ef = ground_velocity_truth - wind_velocity_truth;

% rotate into body frame
rel_vel_bf = Tnb  * rel_vel_ef;

% calculate the true airspeed
TAS = sqrt(rel_vel_bf(1)^2 + rel_vel_bf(2)^2 + rel_vel_bf(3)^2);

% derive the control(disturbance) influence matrix from velocity error to 
% TAS error
G_TAS = jacobian(TAS, [ground_velocity_truth;wind_velocity_truth]);

% derive the error matrix
TAS_dist_matrix = diag([vn_var ve_var vd_var vwn_var vwe_var vwd_var]);
Q_TAS = G_TAS*TAS_dist_matrix*transpose(G_TAS);

% save as C code
ccode(Q_TAS,'file','Q_TAS.c');

save temp.mat;

%% calculate AoA error equations
clear all;
reset(symengine);

load temp.mat;

AoA = atan(rel_vel_bf(3) / rel_vel_bf(1));

% derive the control(disturbance) influence matrix from velocity error to 
% AoA error
G_AoA = jacobian(AoA, [ground_velocity_truth;wind_velocity_truth]);

% derive the error matrix
AoA_dist_matrix = diag([vn_var ve_var vd_var vwn_var vwe_var vwd_var]);
Q_AoA = G_AoA*AoA_dist_matrix*transpose(G_AoA);

% save as C code
ccode(Q_AoA,'file','Q_AoA.c');

save temp.mat;

%% Calculate AoS error equations

clear all;
reset(symengine);

load temp.mat;

AoS = atan(rel_vel_bf(2) / rel_vel_bf(1));
% derive the control(disturbance) influence matrix from velocity error to 
% AoS error
G_AoS = jacobian(AoS, [ground_velocity_truth;wind_velocity_truth]);

% derive the error matrix
AoS_dist_matrix = diag([vn_var ve_var vd_var vwn_var vwe_var vwd_var]);
Q_AoS = G_AoS*AoS_dist_matrix*transpose(G_AoS);

% save as C code
ccode(Q_AoS,'file','Q_AoS.c');

save temp.mat;

%% convert them combined to take advantage of shared terms in the optimiser

clear all;
reset(symengine);

load temp.mat;

% save as C code
ccode([Q_TAS;Q_AoA;Q_AoS],'file','Q_airdata.c');
