% IMPORTANT - This script requires the Matlab symbolic toolbox

% Derivation of quaterion covariance prediction for a rotation about the
% earth frame Z axis and starting at an arbitary orientation. This 4x4 
% matrix can be used to add an additional

% Author:  Paul Riseborough

%% define symbolic variables and constants
clear all;
reset(symengine);
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED
syms daYaw real % earth frame yaw delta angle - rad
syms daYawVar real; % earth frame yaw delta angle variance - rad^2

%% define the state prediction equations

% define the quaternion rotation vector for the state estimate
quat = [q0;q1;q2;q3];

% derive the truth body to nav direction cosine matrix
Tbn = Quat2Tbn(quat);

% define the yaw rotation delta angle in body frame
dAngMeas = transpose(Tbn) * [0; 0; daYaw];

% define the attitude update equations
% use a first order expansion of rotation to calculate the quaternion increment
% acceptable for propagation of covariances
deltaQuat = [1;
    0.5*dAngMeas(1);
    0.5*dAngMeas(2);
    0.5*dAngMeas(3);
    ];
quatNew = QuatMult(quat,deltaQuat);

% Define the state vector & number of states
stateVector = quat;
nStates=numel(stateVector);

% Define vector of process equations
newStateVector = quatNew;

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

% derive the control(disturbance) influence matrix from IMU noise to state
% noise
G = jacobian(newStateVector, daYaw);
[G,SG]=OptimiseAlgebra(G,'SG');

% derive the state error matrix
distMatrix = diag(daYawVar);
Q = G*distMatrix*transpose(G);
[Q,SQ]=OptimiseAlgebra(Q,'SQ');

% set the yaw delta angle to zero - we only needed it to determine the error
% propagation
SF = subs(SF, daYaw, 0);
SG = subs(SG, daYaw, 0);
SQ = subs(SQ, daYaw, 0);

% Derive the predicted covariance matrix using the standard equation
PP = F*P*transpose(F) + Q;
PP = subs(PP, daYaw, 0);

% Collect common expressions to optimise processing
[PP,SPP]=OptimiseAlgebra(PP,'SPP');

save('StateAndCovariancePrediction.mat');
clear all;
reset(symengine);

%% Save output and convert to m and c code fragments

% load equations for predictions and updates
load('StateAndCovariancePrediction.mat');

fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
save(fileName);
SaveScriptCode(nStates);
ConvertToM(nStates);
ConvertToC(nStates);
