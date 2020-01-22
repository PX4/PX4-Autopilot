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

% State vector:
% attitude quaternion
% Velocity - m/sec (North, East, Down)
% Position - m (North, East, Down)
% Delta Angle bias - rad (X,Y,Z)
% Delta Velocity bias - m/s (X,Y,Z)
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
syms dvx_b dvy_b dvz_b real % delta velocity bias - m/sec
syms dt real % IMU time step - sec
syms gravity real % gravity  - m/sec^2
syms daxVar dayVar dazVar dvxVar dvyVar dvzVar real; % IMU delta angle and delta velocity measurement variances
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
syms decl real; % earth magnetic field declination from true north
syms R_DECL R_YAW real; % variance of declination or yaw angle observation
syms BCXinv BCYinv real % inverse of ballistic coefficient for wind relative movement along the x and y  body axes
syms rho real % air density (kg/m^3)
syms R_ACC real % variance of accelerometer measurements (m/s^2)^2
syms Kaccx Kaccy real % derivative of X and Y body specific forces wrt component of true airspeed along each axis (1/s)

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
newStateVector = [quatNew;vNew;pNew;dAngBiasNew;dVelBiasNew;magNnew;magEnew;magDnew;magXnew;magYnew;magZnew;vwnNew;vweNew];

% derive the state transition matrix
F = jacobian(newStateVector, stateVector);
% set the rotation error states to zero
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

% Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed. 

% derive the control(disturbance) influence matrix from IMu noise to state
% noise
G = jacobian(newStateVector, [dAngMeas;dVelMeas]);
[G,SG]=OptimiseAlgebra(G,'SG');

% derive the state error matrix
distMatrix = diag([daxVar dayVar dazVar dvxVar dvyVar dvzVar]);
Q = G*distMatrix*transpose(G);
[Q,SQ]=OptimiseAlgebra(Q,'SQ');

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
Vbw = transpose(Tbn)*[(vn-vwn);(ve-vwe);vd];
% calculate predicted angle of sideslip using small angle assumption
BetaPred = Vbw(2)/Vbw(1);
H_BETA = jacobian(BetaPred,stateVector); % measurement Jacobian
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

% Range is defined as distance from camera focal point to object measured
% along sensor Z axis
syms range real;

% Define rotation matrix from body to sensor frame
syms Tbs_a_x Tbs_a_y Tbs_a_z real;
syms Tbs_b_x Tbs_b_y Tbs_b_z real;
syms Tbs_c_x Tbs_c_y Tbs_c_z real;
Tbs = [ ...
    Tbs_a_x Tbs_a_y Tbs_a_z ; ...
    Tbs_b_x Tbs_b_y Tbs_b_z ; ...
    Tbs_c_x Tbs_c_y Tbs_c_z ...
    ];

% Calculate earth relative velocity in a non-rotating sensor frame
relVelSensor = Tbs * transpose(Tbn) * [vn;ve;vd];

% Divide by range to get predicted angular LOS rates relative to X and Y
% axes. Note these are rates in a non-rotating sensor frame
losRateSensorX = +relVelSensor(2)/range;
losRateSensorY = -relVelSensor(1)/range;

save('temp1.mat','losRateSensorX','losRateSensorY');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');

% calculate the observation Jacobian and Kalman gain for the X axis
H_LOSX = jacobian(losRateSensorX,stateVector); % measurement Jacobian
H_LOSX = simplify(H_LOSX);
K_LOSX = (P*transpose(H_LOSX))/(H_LOSX*P*transpose(H_LOSX) + R_LOS); % Kalman gain vector
K_LOSX = simplify(K_LOSX);
save('temp2.mat','H_LOSX','K_LOSX');
ccode([H_LOSX;transpose(K_LOSX)],'file','LOSX.c');
fix_c_code('LOSX.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');

% calculate the observation Jacobian for the Y axis
H_LOSY = jacobian(losRateSensorY,stateVector); % measurement Jacobian
H_LOSY = simplify(H_LOSY);
K_LOSY = (P*transpose(H_LOSY))/(H_LOSY*P*transpose(H_LOSY) + R_LOS); % Kalman gain vector
K_LOSY = simplify(K_LOSY);
save('temp3.mat','H_LOSY','K_LOSY');
ccode([H_LOSY;transpose(K_LOSY)],'file','LOSY.c');
fix_c_code('LOSY.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for sequential fusion of body frame velocity measurements
load('StatePrediction.mat');

% body frame velocity observations
syms velX velY velZ real;

% velocity observation variance
syms R_VEL real;

% calculate relative velocity in body frame
relVelBody = transpose(Tbn)*[vn;ve;vd];

save('temp1.mat','relVelBody','R_VEL');

% calculate the observation Jacobian for the X axis
H_VELX = jacobian(relVelBody(1),stateVector); % measurement Jacobian
H_VELX = simplify(H_VELX);
save('temp2.mat','H_VELX');
ccode(H_VELX,'file','H_VELX.c');
fix_c_code('H_VELX.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');

% calculate the observation Jacobian for the Y axis
H_VELY = jacobian(relVelBody(2),stateVector); % measurement Jacobian
H_VELY = simplify(H_VELY);
save('temp3.mat','H_VELY');
ccode(H_VELY,'file','H_VELY.c');
fix_c_code('H_VELY.c');

clear all;
reset(symengine);
load('StatePrediction.mat');
load('temp1.mat');

% calculate the observation Jacobian for the Z axis
H_VELZ = jacobian(relVelBody(3),stateVector); % measurement Jacobian
H_VELZ = simplify(H_VELZ);
save('temp4.mat','H_VELZ');
ccode(H_VELZ,'file','H_VELZ.c');
fix_c_code('H_VELZ.c');

clear all;
reset(symengine);

% calculate Kalman gain vector for the X axis
load('StatePrediction.mat');
load('temp1.mat');
load('temp2.mat');

K_VELX = (P*transpose(H_VELX))/(H_VELX*P*transpose(H_VELX) + R_VEL); % Kalman gain vector
K_VELX = simplify(K_VELX);
ccode(K_VELX,'file','K_VELX.c');
fix_c_code('K_VELX.c');

clear all;
reset(symengine);

% calculate Kalman gain vector for the Y axis
load('StatePrediction.mat');
load('temp1.mat');
load('temp3.mat');

K_VELY = (P*transpose(H_VELY))/(H_VELY*P*transpose(H_VELY) + R_VEL); % Kalman gain vector
K_VELY = simplify(K_VELY);
ccode(K_VELY,'file','K_VELY.c');
fix_c_code('K_VELY.c');

clear all;
reset(symengine);

% calculate Kalman gain vector for the Z axis
load('StatePrediction.mat');
load('temp1.mat');
load('temp4.mat');

K_VELZ = (P*transpose(H_VELZ))/(H_VELZ*P*transpose(H_VELZ) + R_VEL); % Kalman gain vector
K_VELZ = simplify(K_VELZ);
ccode(K_VELZ,'file','K_VELZ.c');
fix_c_code('K_VELZ.c');

% reset workspace
clear all;
reset(symengine);

% calculate Kalman gains vectors for X,Y,Z to take advantage of common
% terms
load('StatePrediction.mat');
load('temp1.mat');
load('temp2.mat');
load('temp3.mat');
load('temp4.mat');
K_VELX = (P*transpose(H_VELX))/(H_VELX*P*transpose(H_VELX) + R_VEL); % Kalman gain vector
K_VELY = (P*transpose(H_VELY))/(H_VELY*P*transpose(H_VELY) + R_VEL); % Kalman gain vector
K_VELZ = (P*transpose(H_VELZ))/(H_VELZ*P*transpose(H_VELZ) + R_VEL); % Kalman gain vector
K_VEL = simplify([K_VELX,K_VELY,K_VELZ]);
ccode(K_VEL,'file','K_VEL.c');
fix_c_code('K_VEL.c');


%% derive equations for fusion of 321 sequence yaw measurement
load('StatePrediction.mat');

% Calculate the yaw (first rotation) angle from the 321 rotation sequence
% Provide alternative angle that avoids singularity at +-pi/2 
angMeasA = atan(Tbn(2,1)/Tbn(1,1));
angMeasB = pi/2 - atan(Tbn(1,1)/Tbn(2,1));
H_YAW321 = jacobian([angMeasA;angMeasB],stateVector); % measurement Jacobian
H_YAW321 = simplify(H_YAW321);
ccode(H_YAW321,'file','calcH_YAW321.c');
fix_c_code('calcH_YAW321.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of 312 sequence yaw measurement
load('StatePrediction.mat');

% Calculate the yaw (first rotation) angle from an Euler 312 sequence
% Provide alternative angle that avoids singularity at +-pi/2 
angMeasA = atan(-Tbn(1,2)/Tbn(2,2));
angMeasB = pi/2 - atan(-Tbn(2,2)/Tbn(1,2));
H_YAW312 = jacobian([angMeasA;angMeasB],stateVector); % measurement Jacobian
H_YAW312 = simplify(H_YAW312);
ccode(H_YAW312,'file','calcH_YAW312.c');
fix_c_code('calcH_YAW312.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of dual antenna yaw measurement
load('StatePrediction.mat');

syms ant_yaw real; % yaw angle of antenna array axis wrt X body axis

% define antenna vector in body frame
ant_vec_bf = [cos(ant_yaw);sin(ant_yaw);0];

% rotate into earth frame
ant_vec_ef = Tbn * ant_vec_bf;

% Calculate the yaw angle from the projection
angMeas = atan(ant_vec_ef(2)/ant_vec_ef(1));

H_YAWGPS = jacobian(angMeas,stateVector); % measurement Jacobian
H_YAWGPS = simplify(H_YAWGPS);
ccode(H_YAWGPS,'file','calcH_YAWGPS.c');
fix_c_code('calcH_YAWGPS.c');

% reset workspace
clear all;
reset(symengine);

%% derive equations for fusion of declination
load('StatePrediction.mat');

% the predicted measurement is the angle wrt magnetic north of the horizontal
% component of the measured field
angMeas = atan(magE/magN);
H_MAGD = jacobian(angMeas,stateVector); % measurement Jacobian
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
H_ACCX = simplify(H_ACCX);
[H_ACCX,SH_ACCX]=OptimiseAlgebra(H_ACCX,'SH_ACCX'); % optimise processing
K_ACCX = (P*transpose(H_ACCX))/(H_ACCX*P*transpose(H_ACCX) + R_ACC);
[K_ACCX,SK_ACCX]=OptimiseAlgebra(K_ACCX,'SK_ACCX'); % Kalman gain vector

% Derive observation Jacobian and Kalman gain matrix for Y accel fusion
H_ACCY = jacobian(accYpred,stateVector); % measurement Jacobian
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
ConvertToM(nStates); % convert symbolic expressions to Matlab expressions
ConvertToC(nStates); % convert Matlab expressions to C code expressions
ConvertCtoC(nStates); % convert covariance matrix expressions from array to matrix syntax