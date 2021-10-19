%% define symbolic variables and constants
clear all;
reset(symengine);
syms dax day daz real % IMU delta angle measurements in body axes - rad
syms dvx dvy dvz real % IMU delta velocity measurements in body axes - m/sec
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms vn ve vd real % NED velocity - m/sec
syms pn pe pd real % NED position - m
syms dax_b day_b daz_b real % delta angle bias - rad
syms dvx_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxVar dayVar dazVar dvxVar dvyVar dvzVar real; % IMU delta angle and delta velocity measurement variances
syms vwn vwe real; % NE wind velocity - m/sec
syms magX magY magZ real; % XYZ body fixed magnetic field measurements - milligauss
syms magN magE magD real; % NED earth fixed magnetic field components - milligauss
syms R_MAG real  % variance for magnetic flux measurements - milligauss^2

%% define the state prediction equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the IMU bias errors and scale factor
dAngBias = [dax_b; day_b; daz_b];
dVelBias = [dvx_b; dvy_b; dvz_b];

% define the quaternion rotation vector for the state estimate
quat = [q0;q1;q2;q3];
% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(quat);

% define the truth delta angle
% ignore coning compensation as these effects are negligible in terms of
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas - dAngBias;

% Define the truth delta velocity -ignore sculling and transport rate
% corrections as these negligible are in terms of covariance growth for our
% application and grade of sensor
dVelTruth = dVelMeas - dVelBias;

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngTruth(1);
    0.5*dAngTruth(2);
    0.5*dAngTruth(3);
    ];
quatNew = QuatMult(quat,deltaQuat);

% define the velocity update equations
% ignore coriolis terms for linearisation purposes
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tbn*dVelTruth;

% define the position update equations
pNew = [pn;pe;pd] + [vn;ve;vd]*dt;

% define the IMU error update equations
dAngBiasNew = dAngBias;
dVelBiasNew = dVelBias;

% define the wind velocity update equations
vwnNew = vwn;
vweNew = vwe;

% define the earth magnetic field update equations
magNnew = magN;
magEnew = magE;
magDnew = magD;

% define the body magnetic field update equations
magXnew = magX;
magYnew = magY;
magZnew = magZ;

% Define the state vector & number of states
stateVector = [quat;vn;ve;vd;pn;pe;pd;dAngBias;dVelBias;magN;magE;magD;magX;magY;magZ;vwn;vwe];
nStates=numel(stateVector);

% Define vector of process equations
stateVectorNew = [quatNew;vNew;pNew;dAngBiasNew;dVelBiasNew;magNnew;magEnew;magDnew;magXnew;magYnew;magZnew;vwnNew;vweNew];

%% derive the state transition and state error matrix

% Define the control (disturbance) vector. Error growth in the inertial
% solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. This is OK because we
% have sensor bias accounted for in the state equations.
distVector = [daxVar;dayVar;dazVar;dvxVar;dvyVar;dvzVar];

% derive the control(disturbance) influence matrix
G = jacobian(stateVectorNew, [dAngMeas;dVelMeas]);

% derive the state error matrix
distMatrix = diag(distVector);
Q = G*distMatrix*transpose(G);
f = matlabFunction(Q,'file','calcQ24.m');

% derive the state transition matrix
F = jacobian(stateVectorNew, stateVector);
f = matlabFunction(F,'file','calcF24.m');

%% derive equations for fusion of magnetometer measurements
% rotate earth field into body axes
magMeas = transpose(Tbn)*[magN;magE;magD] + [magX;magY;magZ];

magMeasX = magMeas(1);
H_MAGX = jacobian(magMeasX,stateVector); % measurement Jacobian
f = matlabFunction(H_MAGX,'file','calcH_MAGX.m');

magMeasY = magMeas(2);
H_MAGY = jacobian(magMeasY,stateVector); % measurement Jacobian
f = matlabFunction(H_MAGY,'file','calcH_MAGY.m');

magMeasZ = magMeas(3);
H_MAGZ = jacobian(magMeasZ,stateVector); % measurement Jacobian
f = matlabFunction(H_MAGZ,'file','calcH_MAGZ.m');

%% derive equations for fusion of synthetic deviation measurement
% used to keep correct heading when operating without absolute position or
% velocity measurements - eg when using optical flow

% rotate magnetic field into earth axes
magMeasNED = [magN;magE;magD];

% the predicted measurement is the angle wrt magnetic north of the horizontal
% component of the measured field
angMeas = atan(magMeasNED(2)/magMeasNED(1));
H_MAGD = jacobian(angMeas,stateVector); % measurement Jacobian
H_MAGD = simplify(H_MAGD);

f = matlabFunction(H_MAGD,'file','calcH_MAGD.m');

%% derive equations for fusion of a single magneic compass  heading measurement

% rotate body measured field into earth axes
magMeasNED = Tbn*[magX;magY;magZ];

% the predicted measurement is the angle wrt true north of the horizontal
% component of the measured field
angMeas = atan(magMeasNED(2)/magMeasNED(1));
H_MAG = jacobian(angMeas,stateVector); % measurement Jacobian
f = matlabFunction(H_MAG,'file','calcH_HDG.m');

%% derive equations for sequential fusion of optical flow measurements

% range is defined as distance from camera focal point to centre of sensor fov
syms range real;

% calculate relative velocity in body frame
relVelBody = transpose(Tbn)*[vn;ve;vd];

% divide by range to get predicted angular LOS rates relative to X and Y
% axes. Note these are body angular rate motion compensated optical flow rates
losRateX = +relVelBody(2)/range;
losRateY = -relVelBody(1)/range;

% calculate the observation Jacobian for the X axis
H_LOSX = jacobian(losRateX,stateVector); % measurement Jacobian
H_LOSX = simplify(H_LOSX);
f = matlabFunction(H_LOSX,'file','calcH_LOSX.m');

% calculate the observation Jacobian for the Y axis
H_LOSY = jacobian(losRateY,stateVector); % measurement Jacobian
H_LOSY = simplify(H_LOSY);
f = matlabFunction(H_LOSY,'file','calcH_LOSY.m');

%% derive equations for sequential fusion of body frame velocity measurements

% body frame velocity observations
syms velX velY velZ real;

% velocity observation variance
syms R_VEL real;

% calculate relative velocity in body frame
relVelBody = transpose(Tbn)*[vn;ve;vd];

% calculate the observation Jacobian for the X axis
H_VELX = jacobian(relVelBody(1),stateVector); % measurement Jacobian
H_VELX = simplify(H_VELX);
f = matlabFunction(H_VELX,'file','calcH_VELX.m');

% calculate the observation Jacobian for the Y axis
H_VELY = jacobian(relVelBody(2),stateVector); % measurement Jacobian
H_VELY = simplify(H_VELY);
f = matlabFunction(H_VELY,'file','calcH_VELY.m');

% calculate the observation Jacobian for the Z axis
H_VELZ = jacobian(relVelBody(3),stateVector); % measurement Jacobian
H_VELZ = simplify(H_VELZ);
f = matlabFunction(H_VELZ,'file','calcH_VELZ.m');

%% calculate error transfer matrix for declination error estimate
declination = atan(magE/magN);
T_MAG = jacobian(declination,[magN,magE]);

f = matlabFunction(T_MAG,'file','transfer_matrix.m');

%% calculate Quaternion to Euler angle error transfer matrix
% quat = [q0;q1;q2;q3];
% syms roll pitch yaw 'real';
roll = atan2(2*(quat(3)*quat(4)+quat(1)*quat(2)) , (quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4)));
pitch = -asin(2*(quat(2)*quat(4)-quat(1)*quat(3)));
yaw = atan2(2*(quat(2)*quat(3)+quat(1)*quat(4)) , (quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) - quat(4)*quat(4)));
euler = [roll;pitch;yaw];
error_transfer_matrix = jacobian(euler,quat);
matlabFunction(error_transfer_matrix,'file','quat_to_euler_error_transfer_matrix.m');
