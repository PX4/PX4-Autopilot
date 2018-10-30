% define roll, pitch and yaw variables
syms roll pitch yaw 'real'

% Define yransformaton matrices for rotations about the X,Y and Z body axes
Xrot = [1 0 0 ; 0 cos(roll) sin(roll) ; 0 -sin(roll) cos(roll)];
Yrot = [cos(pitch) 0 -sin(pitch) ; 0 1 0 ; sin(pitch) 0 cos(pitch)];
Zrot = [cos(yaw) sin(yaw) 0 ; -sin(yaw) cos(yaw) 0 ; 0 0 1];

% calculate the tranformation matrix from body to navigation frame resulting from
% a rotation in yaw, roll, pitch order
Tbn_312 = transpose(Yrot*Xrot*Zrot)

% convert to c code and save
ccode(Tbn_312,'file','Tbn_312.c');
fix_c_code('Tbn_312.c');

% define the quaternion elements
syms q0 q1 q2 q3 'real'

% calculate the tranformation matrix from body to navigation frame as a
% function of the quaternions
Tbn_quat = Quat2Tbn([q0;q1;q2;q3]);

% collect the y,x terms required for calculation of the yaw angle
yaw_input_312 = [-Tbn_quat(1,2);Tbn_quat(2,2)];

% convert to c code and save
ccode(yaw_input_312,'file','yaw_input_312.c');
fix_c_code('yaw_input_312.c');
