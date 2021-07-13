function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NED velocity innovations (m/s)
    varInnov] ... % NED velocity innovation variance ((m/s)^2)
    = FuseVelocity( ...
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measVel, ... % NED velocity measurements (m/s)
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_OBS) % velocity observation variance (m/s)^2

innovation = zeros(1,3);
varInnov = zeros(1,3);
H = zeros(3,24);

for obsIndex = 1:3
    
    % velocity states start at index 5
    stateIndex = 4 + obsIndex;

    % Calculate the velocity measurement innovation
    innovation(obsIndex) = states(stateIndex) - measVel(obsIndex);
    
    % Calculate the observation Jacobian
    H(obsIndex,stateIndex) = 1;
    
    varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + R_OBS);
    
end

% Apply an innovation consistency check
for obsIndex = 1:3
    
    if (innovation(obsIndex)^2 / (gateSize^2 * varInnov(obsIndex))) > 1.0
        return;
    end
    
end

% Calculate Kalman gains and update states and covariances
for obsIndex = 1:3
    
    % Calculate the Kalman gains
    K = (P*transpose(H(obsIndex,:)))/varInnov(obsIndex);
    
    % Calculate state corrections
    xk = K * innovation(obsIndex);
    
    % Apply the state corrections
    states = states - xk;
    
    % Update the covariance
    P = P - K*H(obsIndex,:)*P;
    
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    P = 0.5*(P + transpose(P));
    
    % ensure diagonals are positive
    for i=1:24
        if P(i,i) < 0
            P(i,i) = 0;
        end
    end
    
end

end