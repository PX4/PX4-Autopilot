function quat = AlignHeading( ...
    quat, ... % quaternion state vector
    magMea, ... % body frame magnetic flux measurements
    declination)  % Estimated magnetic field delination at current location (rad)

% Get the Euler angles and set yaw to zero
euler = QuatToEul(quat);
euler(3) = 0.0;

% calculate the rotation matrix from body to earth frame
quat_zero_yaw = EulToQuat(euler);
Tbn_zero_yaw = Quat2Tbn(quat_zero_yaw);

% Rotate the magnetic field measurement into earth frame
magMeasNED = Tbn_zero_yaw*magMea;

% Use the projection onto the horizontal to calculate the yaw angle
euler(3) = declination - atan2(magMeasNED(2),magMeasNED(1));

% convert to a quaternion
quat = EulToQuat(euler);

end
