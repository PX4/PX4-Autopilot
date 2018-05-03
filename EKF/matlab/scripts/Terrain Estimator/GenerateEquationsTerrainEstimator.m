% IMPORTANT - This script requires the Matlab symbolic toolbox
% Derivation of EKF equations for estimation of terrain height offset
% Author:  Paul Riseborough
% Last Modified: 20 June 2017

% State vector:

% terrain vertical position (ptd)

% Observations:

% line of sight (LOS) angular rate measurements (rel to sensor frame)
% from a downwards looking optical flow sensor measured in rad/sec about
% the X and Y sensor axes. These rates are motion compensated.

% A positive LOS X rate is a RH rotation of the image about the X sensor
% axis, and is produced by either a positive ground relative velocity in 
% the direction of the Y axis.

% A positive LOS Y rate is a RH rotation of the image about the Y sensor
% axis, and is produced by either a negative ground relative velocity in 
% the direction of the X axis.

% Range measurement aligned with the Z body axis (flat earth model assumed)

% Time varying parameters:

% quaternion parameters defining the rotation from navigation to body axes
% NED flight vehicle velocities
% vehicle vertical position

clear all;

%% define symbolic variables and constants
syms vel_x vel_y vel_z real % NED velocity : m/sec
syms R_OPT real % variance of LOS angular rate mesurements : (rad/sec)^2
syms R_RNG real % variance of range finder measurement : m^2
syms stateNoiseVar real % state process noise variance
syms pd real % position of vehicle in down axis : (m)
syms ptd real % position of terrain in down axis : (m)
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms Popt real % state variance
nStates = 1;


%% derive Jacobians for fusion of optical flow measurements
syms vel_x vel_y vel_z real % NED velocity : m/sec
syms pd real % position of vehicle in down axis : (m)
syms ptd real % position of terrain in down axis : (m)
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED

% derive the body to nav direction cosine matrix
Tbn = Quat2Tbn([q0,q1,q2,q3]);

% calculate relative velocity in sensor frame
relVelSensor = transpose(Tbn)*[vel_x;vel_y;vel_z];

% calculate range to centre of flow sensor fov assuming flat earth
range = ((ptd - pd)/Tbn(3,3));

% divide velocity by range to get predicted motion compensated flow rates
optRateX =  relVelSensor(2)/range;
optRateY = -relVelSensor(1)/range;

% calculate the observation jacobians
H_OPTX = jacobian(optRateX,ptd);
H_OPTY = jacobian(optRateY,ptd);
H_OPT = [H_OPTX;H_OPTY];
ccode(H_OPT,'file','H_OPT.c');
fix_c_code('H_OPT.c');

clear all;
reset(symengine)

%% derive Jacobian for fusion of range finder measurements
syms vel_x vel_y vel_z real % NED velocity : m/sec
syms R_RNG real % variance of range finder measurement : m^2
syms pd real % position of vehicle in down axis : (m)
syms ptd real % position of terrain in down axis : (m)
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED

% derive the body to nav direction cosine matrix
Tbn = Quat2Tbn([q0,q1,q2,q3]);

% calculate range assuming flat earth
range = ((ptd - pd)/Tbn(3,3));

% calculate range observation Jacobian
H_RNG = jacobian(range,ptd); % measurement Jacobian
ccode(H_RNG,'file','H_RNG.c');
fix_c_code('H_RNG.c');
