function [...
    states, ... % state vector after fusion of measurements
    P] ... %
    = FuseMagDeclination( ...
    states, ... % predicted states
    P, ... % predicted covariance
    measDec) % magnetic field declination - azimuth angle measured from true north (rad)

magN = states(17);
magE = states(18);

R_MAG = 0.5^2;

H = calcH_MAGD(magE,magN);
varInnov = (H*P*transpose(H) + R_MAG);
Kfusion = (P*transpose(H))/varInnov;

% Calculate the predicted magnetic declination
predDec = atan2(magE,magN);

% Calculate the measurement innovation
innovation = predDec - measDec;

if (innovation > pi)
    innovation = innovation - 2*pi;
elseif (innovation < -pi)
    innovation = innovation + 2*pi;
end

% correct the state vector
states = states - Kfusion * innovation;

% correct the covariance P = P - K*H*P
P = P - Kfusion*H*P;

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