function [xa_apo,Pa_apo,Rot_matrix,eulerAngles,debugOutput]...
    = AttitudeEKF(approx_prediction,use_inertia_matrix,zFlag,dt,z,q_rotSpeed,q_rotAcc,q_acc,q_mag,r_gyro,r_accel,r_mag,J)


%LQG Position Estimator and Controller
% Observer:
%        x[n|n]   = x[n|n-1] + M(y[n] - Cx[n|n-1] - Du[n])
%        x[n+1|n] = Ax[n|n] + Bu[n]
%
% $Author: Tobias Naegeli $    $Date: 2014 $    $Revision: 3 $
%
%
% Arguments:
% approx_prediction: if 1 then the exponential map is approximated with a
% first order taylor approximation. has at the moment not a big influence
% (just 1st or 2nd order approximation) we should change it to rodriquez
% approximation.
% use_inertia_matrix: set to true if you have the inertia matrix J for your
% quadrotor
% xa_apo_k: old state vectotr
% zFlag: if sensor measurement is available [gyro, acc, mag]
% dt: dt in s
% z: measurements [gyro, acc, mag]
% q_rotSpeed: process noise gyro
% q_rotAcc: process noise gyro acceleration
% q_acc: process noise acceleration
% q_mag: process noise magnetometer
% r_gyro: measurement noise gyro
% r_accel: measurement noise accel
% r_mag: measurement noise mag
% J: moment of inertia matrix


% Output:
% xa_apo: updated state vectotr
% Pa_apo: updated state covariance matrix
% Rot_matrix: rotation matrix
% eulerAngles: euler angles
% debugOutput: not used


%% model specific parameters



% compute once the inverse of the Inertia
persistent Ji;
if isempty(Ji)
    Ji=single(inv(J));
end

%% init
persistent x_apo
if(isempty(x_apo))
    gyro_init=single([0;0;0]);
    gyro_acc_init=single([0;0;0]);
    acc_init=single([0;0;-9.81]);
    mag_init=single([1;0;0]);
    x_apo=single([gyro_init;gyro_acc_init;acc_init;mag_init]);
    
end

persistent P_apo
if(isempty(P_apo))
    %     P_apo = single(eye(NSTATES) * 1000);
    P_apo = single(200*ones(12));
end

debugOutput = single(zeros(4,1));

%% copy the states
wx=  x_apo(1);   % x  body angular rate
wy=  x_apo(2);   % y  body angular rate
wz=  x_apo(3);   % z  body angular rate

wax=  x_apo(4);  % x  body angular acceleration
way=  x_apo(5);  % y  body angular acceleration
waz=  x_apo(6);  % z  body angular acceleration

zex=  x_apo(7);  % x  component gravity vector
zey=  x_apo(8);  % y  component gravity vector
zez=  x_apo(9);  % z  component gravity vector

mux=  x_apo(10); % x  component magnetic field vector
muy=  x_apo(11); % y  component magnetic field vector
muz=  x_apo(12); % z  component magnetic field vector




%% prediction section
% compute the apriori state estimate from the previous aposteriori estimate
%body angular accelerations
if (use_inertia_matrix==1)
    wak =[wax;way;waz]+Ji*(-cross([wax;way;waz],J*[wax;way;waz]))*dt;
else
    wak =[wax;way;waz];
end

%body angular rates
wk =[wx;  wy; wz] + dt*wak;

%derivative of the prediction rotation matrix
O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]';

%prediction of the earth z vector
if (approx_prediction==1)
    %e^(Odt)=I+dt*O+dt^2/2!O^2
    % so we do a first order approximation of the exponential map
    zek =(O*dt+single(eye(3)))*[zex;zey;zez];
    
else
    zek =(single(eye(3))+O*dt+dt^2/2*O^2)*[zex;zey;zez];
    %zek =expm2(O*dt)*[zex;zey;zez]; not working because use double
    %precision
end



%prediction of the magnetic vector
if (approx_prediction==1)
    %e^(Odt)=I+dt*O+dt^2/2!O^2
    % so we do a first order approximation of the exponential map
    muk =(O*dt+single(eye(3)))*[mux;muy;muz];
else
     muk =(single(eye(3))+O*dt+dt^2/2*O^2)*[mux;muy;muz];
    %muk =expm2(O*dt)*[mux;muy;muz]; not working because use double
    %precision
end

x_apr=[wk;wak;zek;muk];

% compute the apriori error covariance estimate from the previous
%aposteriori estimate

EZ=[0,zez,-zey;
    -zez,0,zex;
    zey,-zex,0]';
MA=[0,muz,-muy;
    -muz,0,mux;
    muy,-mux,0]';

E=single(eye(3));
Z=single(zeros(3));

A_lin=[ Z,  E,  Z,  Z
    Z,  Z,  Z,  Z
    EZ, Z,  O,  Z
    MA, Z,  Z,  O];

A_lin=eye(12)+A_lin*dt;

%process covariance matrix

persistent Q
if (isempty(Q))
    Q=diag([ q_rotSpeed,q_rotSpeed,q_rotSpeed,...
        q_rotAcc,q_rotAcc,q_rotAcc,...
        q_acc,q_acc,q_acc,...
        q_mag,q_mag,q_mag]);
end

P_apr=A_lin*P_apo*A_lin'+Q;


%% update
if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1
    
%     R=[r_gyro,0,0,0,0,0,0,0,0;
%         0,r_gyro,0,0,0,0,0,0,0;
%         0,0,r_gyro,0,0,0,0,0,0;
%         0,0,0,r_accel,0,0,0,0,0;
%         0,0,0,0,r_accel,0,0,0,0;
%         0,0,0,0,0,r_accel,0,0,0;
%         0,0,0,0,0,0,r_mag,0,0;
%         0,0,0,0,0,0,0,r_mag,0;
%         0,0,0,0,0,0,0,0,r_mag];
     R_v=[r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel,r_mag,r_mag,r_mag];
    %observation matrix
    %[zw;ze;zmk];
    H_k=[  E,     Z,      Z,    Z;
        Z,     Z,      E,    Z;
        Z,     Z,      Z,    E];
    
    y_k=z(1:9)-H_k*x_apr;
    
    
    %S_k=H_k*P_apr*H_k'+R;
     S_k=H_k*P_apr*H_k';
     S_k(1:9+1:end) = S_k(1:9+1:end) + R_v;
    K_k=(P_apr*H_k'/(S_k));
    
    
    x_apo=x_apr+K_k*y_k;
    P_apo=(eye(12)-K_k*H_k)*P_apr;
else
    if zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==0
        
        R=[r_gyro,0,0;
            0,r_gyro,0;
            0,0,r_gyro];
        R_v=[r_gyro,r_gyro,r_gyro];
        %observation matrix
        
        H_k=[  E,     Z,      Z,    Z];
        
        y_k=z(1:3)-H_k(1:3,1:12)*x_apr;
        
       % S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)'+R(1:3,1:3);
        S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)';
        S_k(1:3+1:end) = S_k(1:3+1:end) + R_v;
        K_k=(P_apr*H_k(1:3,1:12)'/(S_k));
        
        
        x_apo=x_apr+K_k*y_k;
        P_apo=(eye(12)-K_k*H_k(1:3,1:12))*P_apr;
    else
        if  zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==0
            
%             R=[r_gyro,0,0,0,0,0;
%                 0,r_gyro,0,0,0,0;
%                 0,0,r_gyro,0,0,0;
%                 0,0,0,r_accel,0,0;
%                 0,0,0,0,r_accel,0;
%                 0,0,0,0,0,r_accel];
            
            R_v=[r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel];
            %observation matrix
            
            H_k=[  E,     Z,      Z,    Z;
                Z,     Z,      E,    Z];
            
            y_k=z(1:6)-H_k(1:6,1:12)*x_apr;
            
           % S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6);
            S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)';
            S_k(1:6+1:end) = S_k(1:6+1:end) + R_v;
            K_k=(P_apr*H_k(1:6,1:12)'/(S_k));
            
            
            x_apo=x_apr+K_k*y_k;
            P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr;
        else
            if  zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==1
%                 R=[r_gyro,0,0,0,0,0;
%                     0,r_gyro,0,0,0,0;
%                     0,0,r_gyro,0,0,0;
%                     0,0,0,r_mag,0,0;
%                     0,0,0,0,r_mag,0;
%                     0,0,0,0,0,r_mag];
                  R_v=[r_gyro,r_gyro,r_gyro,r_mag,r_mag,r_mag];
                %observation matrix
                
                H_k=[  E,     Z,      Z,    Z;
                    Z,     Z,      Z,    E];
                
                y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apr;
                
                %S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6);
                S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)';
                S_k(1:6+1:end) = S_k(1:6+1:end) + R_v;
                K_k=(P_apr*H_k(1:6,1:12)'/(S_k));
                
                
                x_apo=x_apr+K_k*y_k;
                P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr;
            else
                x_apo=x_apr;
                P_apo=P_apr;
            end
        end
    end
end



%% euler anglels extraction
z_n_b = -x_apo(7:9)./norm(x_apo(7:9));
m_n_b = x_apo(10:12)./norm(x_apo(10:12));

y_n_b=cross(z_n_b,m_n_b);
y_n_b=y_n_b./norm(y_n_b);

x_n_b=(cross(y_n_b,z_n_b));
x_n_b=x_n_b./norm(x_n_b);


xa_apo=x_apo;
Pa_apo=P_apo;
% rotation matrix from earth to body system
Rot_matrix=[x_n_b,y_n_b,z_n_b];


phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3));
theta=-asin(Rot_matrix(1,3));
psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1));
eulerAngles=[phi;theta;psi];

