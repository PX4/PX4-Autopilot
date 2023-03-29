Cz_alp = 5.61;
alpha_0 = -0.04;
Cz_delm = 0.13;
Cx_0 = 0.043;
Cm_0 = 0.0135;
Cm_alpha = -2.74;
Cm_q = -38.21;
L = 0.19;
Cm_delm = -0.99;
S = 0.55;
Iy = 1.135;
m = 13.5;
Sprop = 0.2027;
ro = 1.2682;
kMotor = 80;
Cprop = 1;
g = 9.8;
ki = 0.18994/((pi)*2.8956*0.1592);

KV = 0.0659;
KQ = 0.0659;
R = 0.042;
i0 = 1.5;
CQ2 = -0.01664;
CQ1 = 0.00497;
CQ0 = 0.005230;
CT2 = -0.1079;
CT1 = -0.06044;
CT0 = 0.09357;
Di = 0.5;

%Trim calculation
%Trim point calculation
velocity = 25;
v = 25;
h = 50;
Cz_trim= m*g*cos(0)/(0.5*ro*velocity*velocity*S);
alpha_trim = (((Cz_trim + Cz_alp*alpha_0)*(-1*Cm_delm)/Cz_delm) - Cm_0 + (alpha_0*Cm_alpha))/(Cm_alpha + (Cz_alp*(-1*Cm_delm)/Cz_delm));
delta_m_trim = ((alpha_trim - alpha_0)*Cm_alpha + Cm_0)/(-1*Cm_delm);
Cz_trim = Cz_alp*(alpha_trim - alpha_0) + Cz_delm*delta_m_trim;
Cx_trim = Cx_0 + ki*Cz_trim^2;
T_trim =  0.5*ro*velocity*velocity*S*(Cx_trim);
syms np
np_trim = solve(ro*Di^4*CT0*np^2/(4*(pi)^2) + (ro*Di^3*CT1*velocity*np/(2*(pi))) + ro*Di^2*CT2*velocity*velocity == T_trim, np);
np_trim = max(np_trim);

syms c 
a = ro*Di^5*CQ0/(2*(pi))^2;
b = ro*Di^4*CQ1*velocity/(2*(pi)) + KQ*KV/R;

c_trim = solve((-b + sqrt(b*b - 4*a*c))/(2*a) == np_trim,c);
syms Vin
Vin_trim = solve(ro*Di^3*CQ2*velocity*velocity - KQ*Vin/R + KQ*i0 == c_trim,Vin);

delt_trim = double(Vin_trim/44.4);