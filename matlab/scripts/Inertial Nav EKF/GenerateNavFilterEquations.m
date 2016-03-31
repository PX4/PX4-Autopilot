% IMPORTANT - This script requires the Matlab symbolic toolbox and takes ~3 hours to run

% Derivation of Navigation EKF using a local NED earth Tangent Frame and 
% XYZ body fixed frame
% Sequential fusion of velocity and position measurements
% Fusion of true airspeed
% Sequential fusion of magnetic flux measurements
% 24 state architecture.
% IMU data is assumed to arrive at a constant rate with a time step of dt
% IMU delta angle and velocity data are used as control inputs,
% not observations

% Author:  Paul Riseborough

% Based on use of a rotation vector for attitude estimation as described
% here:

% Mark E. Pittelkau.  "Rotation Vector in Attitude Estimation", 
% Journal of Guidance, Control, and Dynamics, Vol. 26, No. 6 (2003), 
% pp. 855-860.

% State vector:
% error rotation vector in body frame (X,Y,Z)
% Velocity - m/sec (North, East, Down)
% Position - m (North, East, Down)
% Delta Angle bias - rad (X,Y,Z)
% Delta Angle scale factor (X,Y,Z)
% Delta Velocity bias - m/s (Z)
% Earth Magnetic Field Vector - (North, East, Down)
% Body Magnetic Field Vector - (X,Y,Z)
% Wind Vector  - m/sec (North,East)

% Observations:
% NED velocity - m/s
% NED position - m
% True airspeed - m/s
% angle of sideslip - rad
% XYZ magnetic flux

% Time varying parameters:
% XYZ delta angle measurements in body axes - rad
% XYZ delta velocity measurements in body axes - m/sec


%% define symbolic variables and constants
clear all;
reset(symengine);
syms dax day daz real % IMU delta angle measurements in body axes - rad
syms dvx dvy dvz real % IMU delta velocity measurements in body axes - m/sec
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms vn ve vd real % NED velocity - m/sec
syms pn pe pd real % NED position - m
syms dax_b day_b daz_b real % delta angle bias - rad
syms dax_s day_s daz_s real % delta angle scale factor
syms dvz_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxNoise dayNoise dazNoise dvxNoise dvyNoise dvzNoise real; % IMU delta angle and delta velocity measurement noise
syms vwn vwe real; % NE wind velocity - m/sec
syms magX magY magZ real; % XYZ body fixed magnetic field measurements - milligauss
syms magN magE magD real; % NED earth fixed magnetic field components - milligauss
syms R_VN R_VE R_VD real % variances for NED velocity measurements - (m/sec)^2
syms R_PN R_PE R_PD real % variances for NED position measurements - m^2
syms R_TAS real  % variance for true airspeed measurement - (m/sec)^2
syms R_MAG real  % variance for magnetic flux measurements - milligauss^2
syms R_BETA real % variance of sidelsip measurements rad^2
syms R_LOS real % variance of LOS angular rate mesurements (rad/sec)^2
syms ptd real % location of terrain in D axis
syms rotErrX rotErrY rotErrZ real; % error rotation vector in body frame
syms decl real; % earth magnetic field declination from true north
syms R_DECL R_YAW real; % variance of declination or yaw angle observation
syms BCXinv BCYinv real % inverse of ballistic coefficient for wind relative movement along the x and y  body axes
syms rho real % air density (kg/m^3)
syms R_ACC real % variance of accelerometer measurements (m/s^2)^2
syms Kaccx Kaccy real % derivative of X and Y body specific forces wrt componenent of true airspeed along each axis (1/s)

%% define the state prediction equations

% define the measured Delta angle and delta velocity vectors
dAngMeas = [dax; day; daz];
dVelMeas = [dvx; dvy; dvz];

% define the IMU bias errors and scale factor
dAngBias = [dax_b; day_b; daz_b];
dAngScale = [dax_s; day_s; daz_s];
dVelBias = [0;0;dvz_b];

% define the quaternion rotation vector for the state estimate
estQuat = [q0;q1;q2;q3];

% define the attitude error rotation vector, where error = truth - estimate
errRotVec = [rotErrX;rotErrY;rotErrZ];

% define the attitude error quaternion using a first order linearisation
errQuat = [1;0.5*errRotVec];

% Define the truth quaternion as the estimate + error
truthQuat = QuatMult(estQuat, errQuat);

% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(truthQuat);

% define the truth delta angle
% ignore coning compensation as these effects are negligible in terms of 
% covariance growth for our application and grade of sensor
dAngTruth = dAngMeas.*dAngScale - dAngBias - [daxNoise;dayNoise;dazNoise];

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngTruth(1);
    0.5*dAngTruth(2);
    0.5*dAngTruth(3);
    ];
truthQuatNew = QuatMult(truthQuat,deltaQuat);
% calculate the updated attitude error quaternion with respect to the previous estimate
errQuatNew = QuatDivide(truthQuatNew,estQuat);
% change to a rotaton vector - this is the error rotation vector updated state
errRotNew = 2 * [errQuatNew(2);errQuatNew(3);errQuatNew(4)];

% Define the truth delta velocity -ignore sculling and transport rate
% corrections as these negligible are in terms of covariance growth for our
% application and grade of sensor
dVelTruth = dVelMeas - dVelBias - [dvxNoise;dvyNoise;dvzNoise];

% define the velocity update equations
% ignore coriolis terms for linearisation purposes
vNew = [vn;ve;vd] + [0;0;gravity]*dt + Tbn*dVelTruth;

% define the position update equations
pNew = [pn;pe;pd] + [vn;ve;vd]*dt;

% define the IMU error update equations
dabNew = [dax_b; day_b; daz_b];
dasNew = [dax_s; day_s; daz_s];
dvbNew = dvz_b;

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
stateVector = [errRotVec;vn;ve;vd;pn;pe;pd;dax_b;day_b;daz_b;dax_s;day_s;daz_s;dvz_b;magN;magE;magD;magX;magY;magZ;vwn;vwe];
nStates=numel(stateVector);

% Define vector of process equations
newStateVector = [errRotNew;vNew;pNew;dabNew;dasNew;dvbNew;magNnew;magEnew;magDnew;magXnew;magYnew;magZnew;vwnNew;vweNew];

% derive the state transition matrix
F = jacobian(newStateVector, stateVector);
% set the rotation error states to zero
F = subs(F, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
[F,SF]=OptimiseAlgebra(F,'SF');

% define a symbolic covariance matrix using strings to represent 
% '_l_' to represent '( '
% '_c_' to represent ,
% '_r_' to represent ')' 
% these can be substituted later to create executable code
for rowIndex = 1:nStates
    for colIndex = 1:nStates
        eval(['syms OP_l_',num2str(rowIndex),'_c_',num2str(colIndex), '_r_ real']);
        eval(['P(',num2str(rowIndex),',',num2str(colIndex), ') = OP_l_',num2str(rowIndex),'_c_',num2str(colIndex),'_r_;']);
    end
end

save 'StatePrediction.mat';

%% derive the covariance prediction equations
% This reduces the number of floating point operations by a factor of 6 or
% more compared to using the standard matrix operations in code

% Define the control (disturbance) vector. Error growth in the inertial
% solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. This is OK becasue we
% have sensor bias accounted for in the state equations.
distVector = [daxNoise;dayNoise;dazNoise;dvxNoise;dvyNoise;dvzNoise];

% derive the control(disturbance) influence matrix
G = jacobian(newStateVector, distVector);
G = subs(G, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
[G,SG]=OptimiseAlgebra(G,'SG');

% derive the state error matrix
distMatrix = diag(distVector.^2);
Q = G*distMatrix*transpose(G);
[Q,SQ]=OptimiseAlgebra(Q,'SQ');

% remove the disturbance noise from the process equations as it is only
% needed when calculating the disturbance influence matrix
vNew = subs(vNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});
errRotNew = subs(errRotNew,{'daxNoise','dayNoise','dazNoise','dvxNoise','dvyNoise','dvzNoise'}, {0,0,0,0,0,0});

% Derive the predicted covariance matrix using the standard equation
PP = F*P*transpose(F) + Q;

% Collect common expressions to optimise processing
[PP,SPP]=OptimiseAlgebra(PP,'SPP');

save('StateAndCovariancePrediction.mat');
clear all;
reset(symengine);

%% derive equations for fusion of true airspeed measurements
load('StatePrediction.mat');
VtasPred = sqrt((vn-vwn)^2 + (ve-vwe)^2 + vd^2); % predicted measurement
H_TAS = jacobian(VtasPred,stateVector); % measurement Jacobian
H_TAS = subs(H_TAS, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
[H_TAS,SH_TAS]=OptimiseAlgebra(H_TAS,'SH_TAS'); % optimise processing
K_TAS = (P*transpose(H_TAS))/(H_TAS*P*transpose(H_TAS) + R_TAS);
[K_TAS,SK_TAS]=OptimiseAlgebra(K_TAS,'SK_TAS'); % Kalman gain vector

% save equations and reset workspace
save('Airspeed.mat','SH_TAS','H_TAS','SK_TAS','K_TAS');
clear all;
reset(symengine);

%% derive equations for fusion of angle of sideslip measurements
load('StatePrediction.mat');

% calculate wind relative velocities in nav frame and rotate into body frame
Vbw = Tbn'*[(vn-vwn);(ve-vwe);vd];
% calculate predicted angle of sideslip using small angle assumption
BetaPred = Vbw(2)/Vbw(1);
H_BETA = jacobian(BetaPred,stateVector); % measurement Jacobian
H_BETA = subs(H_BETA, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
[H_BETA,SH_BETA]=OptimiseAlgebra(H_BETA,'SH_BETA'); % optimise processing
K_BETA = (P*transpose(H_BETA))/(H_BETA*P*transpose(H_BETA) + R_BETA);[K_BETA,SK_BETA]=OptimiseAlgebra(K_BETA,'SK_BETA'); % Kalman gain vector

% save equations and reset workspace
save('Sideslip.mat','SH_BETA','H_BETA','SK_BETA','K_BETA');
clear all;
reset(symengine);

%% derive equations for fusion of magnetic field measurement
load('StatePrediction.mat');

magMeas = transpose(Tbn)*[magN;magE;magD] + [magX;magY;magZ]; % predicted measurement
H_MAG = jacobian(magMeas,stateVector); % measurement Jacobian
H_MAG = subs(H_MAG, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
[H_MAG,SH_MAG]=OptimiseAlgebra(H_MAG,'SH_MAG');

K_MX = (P*transpose(H_MAG(1,:)))/(H_MAG(1,:)*P*transpose(H_MAG(1,:)) + R_MAG); % Kalman gain vector
[K_MX,SK_MX]=OptimiseAlgebra(K_MX,'SK_MX');
K_MY = (P*transpose(H_MAG(2,:)))/(H_MAG(2,:)*P*transpose(H_MAG(2,:)) + R_MAG); % Kalman gain vector
[K_MY,SK_MY]=OptimiseAlgebra(K_MY,'SK_MY');
K_MZ = (P*transpose(H_MAG(3,:)))/(H_MAG(3,:)*P*transpose(H_MAG(3,:)) + R_MAG); % Kalman gain vector
[K_MZ,SK_MZ]=OptimiseAlgebra(K_MZ,'SK_MZ');

% save equations and reset workspace
save('Magnetometer.mat','SH_MAG','H_MAG','SK_MX','K_MX','SK_MY','K_MY','SK_MZ','K_MZ');
clear all;
reset(symengine);

%% derive equations for sequential fusion of optical flow measurements
load('StatePrediction.mat');

% range is defined as distance from camera focal point to centre of sensor fov
syms range real;

% calculate relative velocity in body frame
relVelBody = transpose(Tbn)*[vn;ve;vd];

% divide by range to get predicted angular LOS rates relative to X and Y
% axes. Note these are body angular rate motion compensated optical flow rates
losRateX = +relVelBody(2)/range;
losRateY = -relVelBody(1)/range;

save('temp1.mat','losRateX','losRateY');

% calculate the observation Jacobian for the X axis
H_LOSX = jacobian(losRateX,stateVector); % measurement Jacobian
H_LOSX = subs(H_LOSX, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_LOSX = simplify(H_LOSX);
save('temp2.mat','H_LOSX');
ccode(H_LOSX,'file','H_LOSX.c');
fix_c_code('H_LOSX.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');

% calculate the observation Jacobian for the Y axis
H_LOSY = jacobian(losRateY,stateVector); % measurement Jacobian
H_LOSY = subs(H_LOSY, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_LOSY = simplify(H_LOSY);
save('temp3.mat','H_LOSY');
ccode(H_LOSY,'file','H_LOSY.c');
fix_c_code('H_LOSY.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');
load('temp2.mat');

% calculate Kalman gain vector for the X axis
K_LOSX = (P*transpose(H_LOSX))/(H_LOSX*P*transpose(H_LOSX) + R_LOS); % Kalman gain vector
K_LOSX = subs(K_LOSX, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
K_LOSX = simplify(K_LOSX);
ccode(K_LOSX,'file','K_LOSX.c');
fix_c_code('K_LOSX.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');
load('temp3.mat');

% calculate Kalman gain vector for the Y axis
K_LOSY = (P*transpose(H_LOSY))/(H_LOSY*P*transpose(H_LOSY) + R_LOS); % Kalman gain vector
K_LOSY = subs(K_LOSY, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
K_LOSY = simplify(K_LOSY);
ccode(K_LOSY,'file','K_LOSY.c');
fix_c_code('K_LOSY.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of 321 sequence yaw measurement
load('StatePrediction.mat');

% Calculate the yaw (first rotation) angle from the 321 rotation sequence
angMeas = atan(Tbn(2,1)/Tbn(1,1));
H_YAW321 = jacobian(angMeas,stateVector); % measurement Jacobian
H_YAW321 = subs(H_YAW321, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_YAW321 = simplify(H_YAW321);
ccode(H_YAW321,'file','calcH_YAW321.c');
fix_c_code('calcH_YAW321.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of 312 sequence yaw measurement
load('StatePrediction.mat');

% Calculate the yaw (first rotation) angle from an Euler 312 sequence
angMeas = atan(-Tbn(1,2)/Tbn(2,2));
H_YAW312 = jacobian(angMeas,stateVector); % measurement Jacobianclea
H_YAW312 = subs(H_YAW312, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_YAW312 = simplify(H_YAW312);
ccode(H_YAW312,'file','calcH_YAW312.c');
fix_c_code('calcH_YAW312.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of declination
load('StatePrediction.mat');

% the predicted measurement is the angle wrt magnetic north of the horizontal
% component of the measured field
angMeas = atan(magE/magN);
H_MAGD = jacobian(angMeas,stateVector); % measurement Jacobian
H_MAGD = subs(H_MAGD, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_MAGD = simplify(H_MAGD);
K_MAGD = (P*transpose(H_MAGD))/(H_MAGD*P*transpose(H_MAGD) + R_DECL);
K_MAGD = simplify(K_MAGD);
ccode([K_MAGD,H_MAGD'],'file','calcMAGD.c');
fix_c_code('calcMAGD.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of lateral body acceleration (multirotors only)
load('StatePrediction.mat');

% use relationship between airspeed along the X and Y body axis and the
% drag to predict the lateral acceleration for a multirotor vehicle type
% where propulsion forces are generated primarily along the Z body axis

vrel = transpose(Tbn)*[(vn-vwn);(ve-vwe);vd]; % predicted wind relative velocity

% calculate drag assuming flight along axis in positive direction
% sign change will be looked after in implementation rather than by adding
% sign functions to symbolic derivation which genererates output with dirac
% functions
% accXpred = -0.5*rho*vrel(1)*vrel(1)*BCXinv; % predicted acceleration measured along X body axis
% accYpred = -0.5*rho*vrel(2)*vrel(2)*BCYinv; % predicted acceleration measured along Y body axis

% Use a simple viscous drag model for the linear estimator equations
% Use the the derivative from speed to acceleration averaged across the 
% speed range
% The nonlinear equation will be used to calculate the predicted
% measurement in implementation
accXpred = -Kaccx*vrel(1); % predicted acceleration measured along X body axis
accYpred = -Kaccy*vrel(2); % predicted acceleration measured along Y body axis

% Derive observation Jacobian and Kalman gain matrix for X accel fusion
H_ACCX = jacobian(accXpred,stateVector); % measurement Jacobian
H_ACCX = subs(H_ACCX, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_ACCX = simplify(H_ACCX);
[H_ACCX,SH_ACCX]=OptimiseAlgebra(H_ACCX,'SH_ACCX'); % optimise processing
K_ACCX = (P*transpose(H_ACCX))/(H_ACCX*P*transpose(H_ACCX) + R_ACC);
[K_ACCX,SK_ACCX]=OptimiseAlgebra(K_ACCX,'SK_ACCX'); % Kalman gain vector

% Derive observation Jacobian and Kalman gain matrix for Y accel fusion
H_ACCY = jacobian(accYpred,stateVector); % measurement Jacobian
H_ACCY = subs(H_ACCY, {'rotErrX', 'rotErrY', 'rotErrZ'}, {0,0,0});
H_ACCY = simplify(H_ACCY);
[H_ACCY,SH_ACCY]=OptimiseAlgebra(H_ACCY,'SH_ACCY'); % optimise processing
K_ACCY = (P*transpose(H_ACCY))/(H_ACCY*P*transpose(H_ACCY) + R_ACC);
[K_ACCY,SK_ACCY]=OptimiseAlgebra(K_ACCY,'SK_ACCY'); % Kalman gain vector

% save equations and reset workspace
save('Drag.mat','SH_ACCX','H_ACCX','SK_ACCX','K_ACCX','SH_ACCY','H_ACCY','SK_ACCY','K_ACCY');
clear all;
reset(symengine);

%% Save output and convert to m and c code fragments

% load equations for predictions and updates
load('StateAndCovariancePrediction.mat');
load('Airspeed.mat');
load('Sideslip.mat');
load('Magnetometer.mat');
load('Drag.mat');

fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
save(fileName);
SaveScriptCode(nStates);
ConvertToM(nStates);
ConvertToC(nStates);