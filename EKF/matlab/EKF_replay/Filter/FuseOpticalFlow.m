function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation, ... % XY optical flow innovations - rad/sec
    varInnov] ... % XY optical flow innovation variances (rad/sec)^2
    = FuseOpticalFlow( ...
    states, ... % predicted states
    P, ... % predicted covariance
    flowRate, ... % XY axis optical flow rate (rad/sec)
    bodyRate, ... % XY axis body rate (rad/sec)
    range,  ... % range from lens to ground measured along the centre of the optical flow sensor field of view
    flowObsVar, ... % flow observation variance - (rad/sec)^2
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

% Calculate predicted angular LOS rates about body frame axes
Tbn = Quat2Tbn(states(1:4));
relVelBody = transpose(Tbn)*[vn;ve;vd];
losRatePred(1) = +relVelBody(2)/range;
losRatePred(2) = -relVelBody(1)/range;

% Calculate measured LOS angular rates using body motion corrected flow
% measurements
losRateMea = - flowRate + bodyRate;

% calculate the observation jacobian, innovation variance and innovation
for obsIndex = 1:2
    
    % Calculate corrections using X component
    if (obsIndex == 1)
        H(1,:) = calcH_LOSX(q0,q1,q2,q3,range,vd,ve,vn);
    elseif (obsIndex == 2)
        H(2,:) = calcH_LOSY(q0,q1,q2,q3,range,vd,ve,vn);
    end
    varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + flowObsVar);
    innovation(obsIndex) = losRatePred(obsIndex) - losRateMea(obsIndex);
end

% check innovations for consistency and exit if they fail the test
for obsIndex = 1:2
    if (innovation(obsIndex)^2 / (varInnov(obsIndex) * gateSize^2) > 1.0);
        return;
    end
end

% calculate the kalman gains and perform the state and covariance update
% using sequential fusion
for obsIndex = 1:2

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