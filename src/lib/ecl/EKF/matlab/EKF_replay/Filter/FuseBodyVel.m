function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation, ... % XY optical flow innovations - rad/sec
    varInnov] ... % XY optical flow innovation variances (rad/sec)^2
    = FuseBodyVel( ...
    states, ... % predicted states
    P, ... % predicted covariance
    relVelBodyMea, ... % XYZ velocity measured by the camera (m/sec)
    obsVar, ... % velocity variances - (m/sec)^2
    gateSize) % innovation gate size (SD)

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);
vn = states(5);
ve = states(6);
vd = states(7);

innovation = zeros(1,2);
varInnov = zeros(1,2);
H = zeros(2,24);

% Calculate predicted velocity measured in body frame axes
Tbn = Quat2Tbn(states(1:4));
relVelBodyPred = transpose(Tbn)*[vn;ve;vd];

% calculate the observation jacobian, innovation variance and innovation
for obsIndex = 1:3
    
    % Calculate corrections using X component
    if (obsIndex == 1)
        H(1,:) = calcH_VELX(q0,q1,q2,q3,vd,ve,vn);
    elseif (obsIndex == 2)
        H(2,:) = calcH_VELY(q0,q1,q2,q3,vd,ve,vn);
    elseif (obsIndex == 3)
        H(3,:) = calcH_VELZ(q0,q1,q2,q3,vd,ve,vn);
    end
    varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + obsVar);
    innovation(obsIndex) = relVelBodyPred(obsIndex) - relVelBodyMea(obsIndex);
end

% check innovations for consistency and exit if they fail the test
for obsIndex = 1:3
    if (innovation(obsIndex)^2 / (varInnov(obsIndex) * gateSize^2) > 1.0);
        return;
    end
end

% calculate the kalman gains and perform the state and covariance update
% using sequential fusion
for obsIndex = 1:3

    Kfusion = (P*transpose(H(obsIndex,:)))/varInnov(obsIndex);
    
    % correct the state vector
    states = states - Kfusion * innovation(obsIndex);
    
    % normalise the updated quaternion states
    quatMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
    if (quatMag > 1e-12)
        states(1:4) = states(1:4) / quatMag;
    end
    
    % correct the covariance P = P - K*H*P
    P = P - Kfusion*H(obsIndex,:)*P;
    
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    % of the matrix which would cause the filter to blow-up
    P = 0.5*(P + transpose(P));
    
    % ensure diagonals are positive
    for i=1:24
        if P(i,i) < 0
            P(i,i) = 0;
        end
    end
    
end

end