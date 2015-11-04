from __future__ import print_function
from pylab import *
from pprint import pprint

#pylint: disable=all

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

C_nb = array([
    [cosTheta*cosPsi, -cosPhi*sinPsi + sinPhi*sinTheta*cosPsi, sinPhi*sinPsi + cosPhi*sinTheta*cosPsi],
    [cosTheta*sinPsi, cosPhi*cosPsi + sinPhi*sinTheta*sinPsi, -sinPhi*cosPsi + cosPhi*sinTheta*sinPsi],
    [-sinTheta, sinPhi*cosTheta, cosPhi*cosTheta]])
    
print('\nC_nb')
pprint(C_nb)

theta = arcsin(-C_nb[2,0])
phi = arctan(C_nb[2,1]/ C_nb[2,2])
psi = arctan(C_nb[1,0]/ C_nb[0,0])
print('\nphi {:f}, theta {:f}, psi {:f}\n'.format(phi, theta, psi))

q = array([
    cosPhi_2*cosTheta_2*cosPsi_2 + sinPhi_2*sinTheta_2*sinPsi_2,
    sinPhi_2*cosTheta_2*cosPsi_2 - cosPhi_2*sinTheta_2*sinPsi_2,
    cosPhi_2*sinTheta_2*cosPsi_2 + sinPhi_2*cosTheta_2*sinPsi_2,
    cosPhi_2*cosTheta_2*sinPsi_2 - sinPhi_2*sinTheta_2*cosPsi_2])
     
a = q[0]
b = q[1]
c = q[2]
d = q[3]

print('\nq')
pprint(q.T)

a2 = a*a
b2 = b*b
c2 = c*c
d2 = d*d

C2_nb = array([
    [a2 + b2 - c2 - d2, 2*(b*c - a*d), 2*(b*d + a*c)],
    [2*(b*c + a*d), a2 - b2 + c2 - d2, 2*(c*d - a*b)],
    [2*(b*d - a*c), 2*(c*d + a*b), a2 - b2 - c2 + d2]])

print('\nC2_nb')
pprint(C2_nb)

# vim: set et ft=python fenc=utf-8 ff=unix sts=4 sw=4 ts=8 : 
