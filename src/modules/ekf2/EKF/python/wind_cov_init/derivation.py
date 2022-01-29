#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 29 14:11:58 2019

@author: roman
"""

from sympy import *

################## Here are the variables you can change to see the effects on the cov matrix ###########################
yaw_init =  0.5

# ground speed in body frame (comes from ekf2)
groundspeed_body_x_init = 5
groundspeed_body_y_init = 5

# true airspeed measured by pitot tube
V_init = 7

# heading variance
R_yaw_init = rad(15.0)**2

# sideslip variance
R_beta_init = rad(15.0)**2

# True airspeed measurement variance
R_tas_init = 1.4**2

#########################################################################################################################

# define symbols: true airspeed, sidslip angle,
V, beta, yaw, groundspeed_body_x, groundspeed_body_y = symbols('V beta yaw vx_body vy_body')
R_tas, R_beta, R_yaw = symbols('R_tas R_beta R_yaw')


# body x/y component of relative wind vector ( V is what the airspeed sensor measures)
Vx = V * cos(beta)
Vy = V * sin(beta)


# wind in body frame
wind_body_x = groundspeed_body_x - Vx
wind_body_y = groundspeed_body_y - Vy

# wind in earth frame
wind_n = cos(yaw) * wind_body_x - sin(yaw) * wind_body_y
wind_e = sin(yaw) * wind_body_x + cos(yaw) * wind_body_y
wind_earth = Matrix([wind_n, wind_e])

# jacobian of earth wind vector with respect to states with known uncertainties
G = wind_earth.jacobian([V, beta, yaw])

# initial covariance matrix
P = Matrix([[R_tas, 0, 0], [0, R_beta,0], [0,0,R_yaw]])

# earth wind covariance matrix, assume 0 sideslip angle
P_wind_earth = (G*P*G.T).subs([(beta, 0)])

P_wind_earth_numeric = P_wind_earth.subs([(V, V_init),(yaw, yaw_init), (R_tas, R_tas_init), (R_yaw, R_yaw_init), (R_beta, R_beta_init)])
P_wind_earth_numeric = P_wind_earth_numeric.subs([(groundspeed_body_x, groundspeed_body_x_init), (groundspeed_body_y, groundspeed_body_y_init) ])


print('P[22][22] = ' + str(P_wind_earth_numeric[0,0]))
print('P[22][23] = ' + str(P_wind_earth_numeric[0,1]))
print('P[23][22] = ' + str(P_wind_earth_numeric[1,0]))
print('P[23][23] = ' + str(P_wind_earth_numeric[1,1]))
