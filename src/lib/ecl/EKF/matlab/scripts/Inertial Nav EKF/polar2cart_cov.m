clear all;
syms spd yaw real;
syms R_spd R_yaw real;
vx = spd*cos(yaw);
vy = spd*sin(yaw);
Tpc = jacobian([vx;vy],[spd;yaw]);
R_polar = [R_spd 0;0 R_yaw];
R_cartesian = Tpc*R_polar*Tpc';