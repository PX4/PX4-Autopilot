
# coding: utf-8


from __future__ import division, print_function
import pylab as pl

#pylint: disable=no-member, invalid-name

def quat_from_euler(phi, theta, psi):
    "Quaternion from (body 3(psi)-2(theta)-1(phi) euler angles"
    from sympy import cos, sin
    return [
        cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2),
        sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2),
        cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2),
        cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
    ]

def quat_prod(q, r):
    "Quaternion product"
    return [
        r[0]*q[0] - r[1]*q[1] - r[2]*q[2] - r[3]*q[3],
        r[0]*q[1] + r[1]*q[0] - r[2]*q[3] + r[3]*q[2],
        r[0]*q[2] + r[1]*q[3] + r[2]*q[0] - r[3]*q[1],
        r[0]*q[3] - r[1]*q[2] + r[2]*q[1] + r[3]*q[0]
    ]

def quat_to_euler(q):
    "Quaternion to (body 3(psi)-2(theta)-1(phi) euler angles"
    return [
        pl.arctan(2*(q[0]*q[1] +  q[2]*q[3])/(1 - 2*(q[1]**2 + q[2]**2))),
        pl.arcsin(2*(q[0]*q[2] - q[3]*q[1])),
        pl.arctan(2*(q[0]*q[3] +  q[1]*q[2])/(1 - 2*(q[2]**2 + q[3]**2))),

    ]

