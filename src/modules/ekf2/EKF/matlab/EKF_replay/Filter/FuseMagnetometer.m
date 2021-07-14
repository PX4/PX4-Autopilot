function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation, ... % Declination innovation - rad
    varInnov] ... %
    = FuseMagnetometer( ...
    states, ... % predicted states
    P, ... % predicted covariance
    magMea, ... % body frame magnetic flux measurements
    testRatio, ... % Size of magnetometer innovation in standard deviations before measurements are rejected
    R_MAG) % magnetometer measurement variance - gauss^2

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);

magXbias = states(20);
magYbias = states(21);
magZbias = states(22);

magN = states(17);
magE = states(18);
magD = states(19);

innovation = zeros(1,3);
varInnov = zeros(1,3);
H = zeros(3,24);

% Calculate the predicted magnetometer measurement
Tbn = Quat2Tbn(states(1:4));
magPred = transpose(Tbn)*[magN;magE;magD] + [magXbias;magYbias;magZbias];

% calculate the observation jacobian, innovation variance and innovation
for obsIndex = 1:3
    
    % Calculate corrections using X component
    if (obsIndex == 1)
        H(1,:) = calcH_MAGX(magD,magE,magN,q0,q1,q2,q3);
    elseif (obsIndex == 2)
        H(2,:) = calcH_MAGY(magD,magE,magN,q0,q1,q2,q3);
    elseif (obsIndex == 3)
        H(3,:) = calcH_MAGZ(magD,magE,magN,q0,q1,q2,q3);
    end
    varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + R_MAG);
    innovation(obsIndex) = magPred(obsIndex) - magMea(obsIndex);
end

% check innovations for consistency and exit if they fail the test
for obsIndex = 1:3
    if (innovation(obsIndex)^2 / (varInnov(obsIndex) * testRatio^2) > 1.0);
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