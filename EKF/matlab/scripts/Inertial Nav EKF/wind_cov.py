import sympy as sp
Wx, Wy, yaw, R_TAS, initial_wind_var_body_y, R_yaw = sp.symbols('Wx Wy yaw R_TAS initial_wind_var_body_y R_yaw')
Wn = Wx * sp.cos(yaw) - Wy * sp.sin(yaw)
We = Wx * sp.sin(yaw) + Wy * sp.cos(yaw)

Wn_Wx = sp.diff(Wn, Wx)
Wn_Wy = sp.diff(Wn, Wy)
Wn_yaw = sp.diff(Wn, yaw)
We_Wx = sp.diff(We, Wx)
We_Wy = sp.diff(We, Wy)
We_yaw = sp.diff(We, yaw)

G = sp.Matrix([[Wn_Wx, Wn_Wy, Wn_yaw],[We_Wx, We_Wy, We_yaw]])
b_wind_cov = sp.Matrix([[R_TAS, 0.0, 0.0], [0.0,initial_wind_var_body_y, 0.0], [0.0, 0.0, R_yaw]])
i_wind_cov = G * b_wind_cov * G.T

print('P[22][22] = ' + str(i_wind_cov[0,0]))
print('P[22][23] = ' + str(i_wind_cov[0,1]))
print('P[23][22] = ' + str(i_wind_cov[1,0]))
print('P[23][23] = ' + str(i_wind_cov[1,1]))