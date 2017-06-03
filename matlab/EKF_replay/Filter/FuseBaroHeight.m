function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % NE position innovations (m)
    varInnov] ... % NE position innovation variance (m^2)
    = FuseBaroHeight( ...
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measHgt, ... % NE position measurements (m)
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_OBS) % position observation variance (m)^2

H = zeros(1,24);

% position states start at index 8
stateIndex = 10;

% Calculate the vertical position height innovation (posD is opposite
% sign to height)
innovation = states(stateIndex) + measHgt;

% Calculate the observation Jacobian
H(stateIndex) = 1;

varInnov = (H*P*transpose(H) + R_OBS);

% Apply an innovation consistency check
if (innovation^2 / (gateSize^2 * varInnov)) > 1.0
    return;
end

% Calculate Kalman gains and update states and covariances

% Calculate the Kalman gains
K = (P*transpose(H))/varInnov;

% Calculate state corrections
xk = K * innovation;

% Apply the state corrections
states = states - xk;

% Update the covariance
P = P - K*H*P;

% Force symmetry on the covariance matrix to prevent ill-conditioning
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:24
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end