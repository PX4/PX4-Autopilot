phi = 0.1
theta = 0.2
psi = 0.3

cosPhi = cos(phi)
cosPhi_2 = cos(phi/2)
sinPhi = sin(phi)
sinPhi_2 = sin(phi/2)

cosTheta = cos(theta)
cosTheta_2 = cos(theta/2)
sinTheta = sin(theta)
sinTheta_2 = sin(theta/2)

cosPsi = cos(psi)
cosPsi_2 = cos(psi/2)
sinPsi = sin(psi)
sinPsi_2 = sin(psi/2)

C_nb = [cosTheta*cosPsi, -cosPhi*sinPsi + sinPhi*sinTheta*cosPsi, sinPhi*sinPsi + cosPhi*sinTheta*cosPsi;
        cosTheta*sinPsi, cosPhi*cosPsi + sinPhi*sinTheta*sinPsi, -sinPhi*cosPsi + cosPhi*sinTheta*sinPsi;
        -sinTheta, sinPhi*cosTheta, cosPhi*cosTheta]
     
//C_nb = round(C_nb*1e3)
//disp(C_nb)
//C_nb = C_nb/1e3

disp(C_nb)
disp(C_nb(3,1))
disp(sin(theta))
theta = asin(-C_nb(3,1))
phi = atan(C_nb(3,2), C_nb(3,3))
psi = atan(C_nb(2,1), C_nb(1,1))
printf('phi %f\n', phi)
printf('theta %f\n', theta)
printf('psi %f\n', psi)

q = [cosPhi_2*cosTheta_2*cosPsi_2 - sinPhi_2*sinTheta_2*sinPsi_2;
     sinPhi_2*cosTheta_2*cosPsi_2 + cosPhi_2*sinTheta_2*sinPsi_2;
     cosPhi_2*sinTheta_2*cosPsi_2 - sinPhi_2*cosTheta_2*sinPsi_2;
     cosPhi_2*cosTheta_2*sinPsi_2 + sinPhi_2*sinTheta_2*cosPsi_2]
     
printf('q: %f %f %f %f', q(1), q(2), q(3), q(4))
