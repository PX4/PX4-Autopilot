% calculate the variances for an equivalent rotation vector
% inputs are the quaternion orientation and the 4x4 covariance matrix for the quaternions
% output is a vector of variances for the rotation vector that is equivalent to the quaternion
clear all;
reset(symengine);
syms q0 q1 q2 q3 real % quaternions defining attitude of body axes relative to local NED

% define quaternion rotation
quat = [q0;q1;q2;q3];

% convert to a rotation vector
delta = 2*acos(q0);
rotX = delta*(q1/sin(delta/2));
rotY = delta*(q2/sin(delta/2));
rotZ = delta*(q3/sin(delta/2));
rotVec = [rotX;rotY;rotZ];

% calculate transfer matrix from quaternion to rotation vector
G = jacobian(rotVec, quat);

% define a symbolic covariance matrix using strings to represent 
% '_l_' to represent '( '
% '_c_' to represent ,
% '_r_' to represent ')' 
% these can be substituted later to create executable code
for rowIndex = 1:4
    for colIndex = 1:4
        eval(['syms P_l_',num2str(rowIndex-1),'_c_',num2str(colIndex-1), '_r_ real']);
        eval(['quatCovMat(',num2str(rowIndex),',',num2str(colIndex), ') = P_l_',num2str(rowIndex-1),'_c_',num2str(colIndex-1),'_r_;']);
    end
end

% rotate the covariance from quaternion to rotation vector
rotCovMat = G*quatCovMat*transpose(G);

% take the variances
rotVarVec = [rotCovMat(1,1);rotCovMat(2,2);rotCovMat(3,3)];

% convert to c-code
ccode(rotVarVec,'file','rotVarVec.c');
