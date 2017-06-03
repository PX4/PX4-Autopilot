function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NE position innovations (m)
    varInnov] ... % NE position innovation variance (m^2)
    = FusePosition( ...
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measPos, ... % NE position measurements (m)
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_OBS) % position observation variance (m)^2

innovation = zeros(1,2);
varInnov = zeros(1,2);
H = zeros(2,24);

for obsIndex = 1:2
    
    % velocity states start at index 8
    stateIndex = 7 + obsIndex;

    % Calculate the velocity measurement innovation
    innovation(obsIndex) = states(stateIndex) - measPos(obsIndex);
    
    % Calculate the observation Jacobian
    H(obsIndex,stateIndex) = 1;
    
    varInnov(obsIndex) = (H(obsIndex,:)*P*transpose(H(obsIndex,:)) + R_OBS);
    
end

% Apply an innovation consistency check
for obsIndex = 1:2
    
    if (innovation(obsIndex)^2 / (gateSize^2 * varInnov(obsIndex))) > 1.0
        return;
    end
    
end

% Calculate Kalman gains and update states and covariances
for obsIndex = 1:2
    
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