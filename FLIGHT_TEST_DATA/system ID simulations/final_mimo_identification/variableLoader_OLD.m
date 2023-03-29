Cz_alp = 3.45;
alpha_0 = 0.4712;
Cz_delm = -0.36;
Cx_0 = 0.03;
Cm_0 = -0.02338;
Cm_alpha = -0.38;
Cm_q = -3.6;
L = 0.18994;
Cm_delm = -0.5;
S = 0.55;
Iy = 1.135;
m = 13.5;
Sprop = 0.2027;
ro = 1.2682;
kMotor = 80;
Cprop = 1;
g = 9.8;
ki = 0.18994/(pi*2.8956*0.1592);

%Trim point calculation
velocity = 25;
Cz_trim= m*g*cos(0)/(0.5*ro*velocity*velocity*S);
alpha_trim = (((Cz_trim + Cz_alp*alpha_0)*(-1*Cm_delm)/Cz_delm) - Cm_0 + (alpha_0*Cm_alpha))/(Cm_alpha + (Cz_alp*(-1*Cm_delm)/Cz_delm));
delta_m_trim = ((alpha_trim - alpha_0)*Cm_alpha + Cm_0)/(-1*Cm_delm);
Cz_trim = Cz_alp*(alpha_trim - alpha_0) + Cz_delm*delta_m_trim;
Cx_trim = Cx_0 + ki*Cz_trim^2;
T_trim =  0.5*ro*velocity*velocity*S*(Cx_trim);
delt_trim = sqrt((T_trim/(ro*Sprop*Cprop/(2*m))) + velocity*velocity)/kMotor;

