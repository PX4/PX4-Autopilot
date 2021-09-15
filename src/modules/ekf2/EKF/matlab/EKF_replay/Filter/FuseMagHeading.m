function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation, ... % Declination innovation - rad
    varInnov] ... %
    = FuseMagHeading( ...
    states, ... % predicted states
    P, ... % predicted covariance
    magData, ... % XYZ body frame magnetic flux measurements - gauss
    measDec, ... % magnetic field declination - azimuth angle measured from true north (rad)
    innovGate, ... % innovation gate size (SD)
    R_MAG) % magnetic heading measurement variance - rad^2

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);

magX = magData(1);
magY = magData(2);
magZ = magData(3);

H = calcH_HDG(magX,magY,magZ,q0,q1,q2,q3);
varInnov = (H*P*transpose(H) + R_MAG);
Kfusion = (P*transpose(H))/varInnov;

% Calculate the predicted magnetic declination
Tbn = Quat2Tbn(states(1:4));
magMeasNED = Tbn*[magX;magY;magZ];
predDec = atan2(magMeasNED(2),magMeasNED(1));

% Calculate the measurement innovation
innovation = predDec - measDec;

if (innovation > pi)
    innovation = innovation - 2*pi;
elseif (innovation < -pi)
    innovation = innovation + 2*pi;
end

% Apply a innovation consistency check
if (innovation^2 / (innovGate^2 * varInnov)) > 1.0
    innovation = NaN;
    varInnov = NaN;
    return;
end

% correct the state vector
states = states - Kfusion * innovation;

% normalise the updated quaternion states
quatMag = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
if (quatMag > 1e-12)
    states(1:4) = states(1:4) / quatMag;
end

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