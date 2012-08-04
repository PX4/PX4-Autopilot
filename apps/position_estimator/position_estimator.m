function [xapo1,Papo1] = position_estimator(u,z,xapo,Papo,gps_covariance,predict_only) %if predit_onli == 1: no update step: use this when no new gps data is available
%#codegen
%%initialization
%use model F=m*a x''=F/m
% 250Hz---> dT = 0.004s
%u=[phi;theta]
%x=[px;vx;py;vy];
%%------------------------------------------
dT=0.004;
%%------------------------------------------------

%R_t=[1,-r*dT,q*dT;r*dT,1,-p*dT;-q*dT,p*dT,1];


F=[ 1,      0.004,  0,      0,      0,      0;
    0,      1,      0,      0,      0,      0;
    0,      0,      1,      0.004,  0,      0;
    0,      0,      0,      1,      0,      0;
    0,      0,      0,      0,      1,      0.004;
    0,      0,      0,      0,      0,      1];

B=[ 0,      -0.1744;
    0,      -87.2;
    0.1744, 0;
    87.2,   0;
    0,      0;
    0,      0];
 
H=[1,0,0,0,0,0;
   0,0,1,0,0,0;
   0,0,0,0,1,0];



 Q=[1e-007        ,0      ,0      ,0      ,0      ,0;
    0           ,1   ,0      ,0      ,0      ,0;
    0           ,0      ,1e-007   ,0      ,0      ,0;
    0           ,0      ,0      ,1   ,0      ,0
    0           ,0      ,0      ,0      ,1e-007   ,0;
    0           ,0      ,0      ,0      ,0      ,1]; %process Covariance Matrix


R=[gps_covariance(1),  0,          0;
   0,       gps_covariance(2),     0;
   0,       0,          gps_covariance(3)]; %measurement Covariance Matrix

%%prediction

xapri=F*xapo+B*u;
Papri=F*Papo*F'+Q;

if 1 ~= predict_only
    %update
    yr=z-H*xapri;
    S=H*Papri*H'+R;
    K=(Papri*H')/S;
    xapo1=xapri+K*yr;
    Papo1=(eye(6)-K*H)*Papri;
else
    Papo1=Papri;
    xapo1=xapri;
end