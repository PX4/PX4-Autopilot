"""
IEKF for UAV state estimation
"""

# pylint: disable=invalid-name, too-many-instance-attributes
# pylint: disable=unused-argument, no-member, too-many-arguments
# pylint: disable=too-many-locals

from transforms3d.quaternions import rotate_vector, qinverse, qmult
from transforms3d.taitbryan import euler2mat
import numpy as np
from util import BETA_TABLE, skew, vect2quat, \
 X, Y_accel, Y_gps, Y_mag, Y_baro, Y_lidar, params, U, Xe


class RIEKF(object):
    """
    Right invariant extended kalman filter
    source: https://hal.archives-ouvertes.fr/hal-00494342/document

    TODO: handle mag correction only in yaw
    """

    def __init__(self, x0, P0, Q):
        self.x = x0
        self.P0 = P0

        self.K_accel = np.zeros((Xe.n, Y_accel.n))
        self.K_mag = np.zeros((Xe.n, Y_mag.n))
        self.K_gps = np.zeros((Xe.n, Y_gps.n))
        self.K_baro = np.zeros((Xe.n, Y_baro.n))
        self.K_lidar = np.zeros((Xe.n, Y_lidar.n))

        self.P = P0
        self.Q = Q
        self.g_n = [0, 0, -9.8]

        self.set_mag_field(1.0, 0)
        self.J = self.compute_invariants(x0, np.zeros(6))

        self.accel_fault = False
        self.mag_fault = False
        self.gps_fault = False
        self.baro_fault = False
        self.lidar_fault = False

    def set_mag_field(self, incl, decl):
        """
        Set magnetic field vector fixed in navigation frame
        """
        self.B_n = euler2mat(decl, -incl, 0).dot([1, 0, 0])

    @staticmethod
    def compute_invariants(x, u):
        """
        Compute the invariant vector
        """
        q_nb = x[:4]
        # V_n = x[4:7]
        gb_b = x[7:10]
        as_b = x[10]

        omega_nb_b = u[:3]
        a_b = u[3:6]

        J_omega_n = rotate_vector(omega_nb_b - gb_b, q_nb)
        J_a_n = rotate_vector(a_b/as_b, q_nb)

        return np.hstack([J_omega_n, J_a_n])

    def compute_dx(self, K, r):
        """
        Calculate non-linear correction
        """
        x = self.x
        q_nb = x[X.q_nb_0:X.q_nb_3 + 1]
        a_s = x[X.accel_scale]
        q_bn = qinverse(q_nb)

        K_q = K[Xe.rot_bx:Xe.rot_bz + 1, :]
        K_vel = K[Xe.vel_N:Xe.vel_D + 1, :]
        K_omega = K[Xe.gyro_bias_bx:Xe.gyro_bias_bz + 1, :]
        K_a = K[Xe.accel_scale, :]
        K_pos = K[Xe.pos_N:Xe.pos_D + 1, :]
        K_terrain_alt = K[Xe.terrain_alt, :]
        K_baro_bias = K[Xe.baro_bias, :]

        dx = np.zeros(X.n)

        # TODO correct if observable
        dx[X.q_nb_0: X.q_nb_3 + 1] = qmult(vect2quat(K_q.dot(r)), q_nb)
        dx[X.vel_N: X.vel_D + 1] = K_vel.dot(r)
        dx[X.gyro_bias_bx: X.gyro_bias_bz + 1] = \
            rotate_vector(K_omega.dot(r), q_bn)
        dx[X.accel_scale] = a_s*K_a.dot(r)
        dx[X.pos_N: X.pos_D + 1] = K_pos.dot(r)
        dx[X.terrain_alt] = K_terrain_alt.dot(r)
        dx[X.baro_bias] = K_baro_bias.dot(r)
        return dx

    def kalman_correct(self, name, r, H, R):
        """
        Calculate kalman gain and apply correction to state and covariance
        """
        P = self.P
        S = H.dot(P).dot(H.T) + R
        fault = False

        # fault threshold
        beta = r.T.dot(np.linalg.inv(S)).dot(r)
        if beta > BETA_TABLE[len(r)]:
            # print('fault', name, beta)
            fault = True

        K = P.dot(H.T).dot(np.linalg.inv(S))
        dx = self.compute_dx(K, r)
        dP = -K.dot(H).dot(P)
        return dx, dP, K, fault

    def set_P(self, P):
        """
        set P and check bounds
        """
        self.P = P
        self.bound_P()

    def set_x(self, x):
        """
        set x and check bounds
        """
        self.x = x
        self.bound_x()

    def correct_baro(self, y_baro, dt):
        """
        Perform correction step
        """
        x = self.x

        # measurement matrix
        H = np.zeros((Y_baro.n, Xe.n))
        H[Y_baro.asl, Xe.pos_D] = -1
        H[Y_baro.asl, Xe.baro_bias] = 1

        # measurement covariance matrix
        R = params['BARO_Z']**2*np.eye(Y_baro.n)/dt

        # residual
        yh = -x[X.pos_D] + x[X.baro_bias]
        r = np.array([y_baro - yh])

        # kalman correction
        dx, dP, self.K_baro, self.baro_fault = \
            self.kalman_correct('baro', r, H, R)
        self.set_P(self.P + dP)
        self.set_x(self.x + dx)

    def correct_mag(self, y_B_b, dt):
        """
        Perform correction step
        """
        y_B_b_unit = y_B_b/np.linalg.norm(y_B_b)
        B_n_unit = self.B_n/np.linalg.norm(self.B_n)

        x = self.x
        q_nb = x[X.q_nb_0:X.q_nb_3 + 1]

        # measurement matrix
        H = np.zeros((Y_mag.n, Xe.n))
        H[Y_mag.bx:Y_mag.bz + 1, Xe.rot_bx:Xe.rot_bz + 1] = 2*skew(B_n_unit)

        # measurement covariance matrix
        R = np.diag([params['MAG_NE'],
                     params['MAG_NE'],
                     params['MAG_D']])**2/dt
        # residual
        r = rotate_vector(y_B_b_unit, q_nb) - B_n_unit

        # kalman correction
        dx, dP, self.K_mag, self.mag_fault = \
            self.kalman_correct('mag', r, H, R)
        self.set_P(self.P + dP)
        self.set_x(self.x + dx)

    def correct_accel(self, y_A_b, dt):
        """
        Perform correction step
        """
        x = self.x
        q_nb = x[X.q_nb_0:X.q_nb_3 + 1]
        a_s = x[X.accel_scale]

        # measurement matrix
        # TODO check math
        H = np.zeros((Y_accel.n, Xe.n))
        H[Y_accel.bx:Y_accel.bz + 1,
          Xe.rot_bx:Xe.rot_bz + 1] = 2*skew(self.g_n)

        # measurement covariance matrix
        R = params['ACC_C']**2*np.eye(3)/dt

        # residual
        r = rotate_vector(y_A_b/a_s, q_nb) - self.g_n

        # kalman correction
        dx, dP, self.K_accel, self.accel_fault = \
            self.kalman_correct('accel', r, H, R)
        self.set_P(self.P + dP)
        self.set_x(self.x + dx)

    def correct_gps(self, y_gps, dt):
        """
        Perform correction step
        """
        x = self.x
        p_n = x[X.pos_N: X.pos_D + 1]
        V_n = x[X.vel_N: X.vel_D + 1]
        I3 = np.eye(3)

        # measurement matrix
        H = np.zeros((Y_gps.n, Xe.n))
        H[Y_gps.pos_N:Y_gps.pos_D + 1, Xe.pos_N:Xe.pos_D + 1] = I3
        H[Y_gps.vel_N:Y_gps.vel_D + 1, Xe.vel_N:Xe.vel_D + 1] = I3

        # measurement covariance matrix
        R = np.diag([params['GPS_XY'],
                     params['GPS_XY'],
                     params['GPS_Z'],
                     params['GPS_VXY'],
                     params['GPS_VXY'],
                     params['GPS_VZ']])**2/dt
        # residual
        r = y_gps - np.hstack([p_n, V_n])

        # kalman correction
        dx, dP, self.K_gps, self.gps_fault = \
            self.kalman_correct('gps', r, H, R)
        self.set_P(self.P + dP)
        self.set_x(self.x + dx)

    def correct_lidar(self, y_lidar, dt):
        """
        Perform correction step
        """
        x = self.x

        # measurement matrix
        # d = -pos_d - terrain_alt
        H = np.zeros((Y_lidar.n, Xe.n))
        H[Y_lidar.d, Xe.pos_D] = -1
        H[Y_lidar.d, Xe.terrain_alt] = -1

        # measurement covariance matrix
        R = params['LDR_Z']**2*np.eye(Y_lidar.n)/dt

        # residual
        yh = -x[X.pos_D] - x[X.terrain_alt]
        r = np.array([y_lidar - yh])

        # kalman correction
        dx, dP, self.K_lidar, self.lidar_fault = \
            self.kalman_correct('lidar', r, H, R)
        self.set_P(self.P + dP)
        self.set_x(self.x + dx)

    def dynamics(self, x, u):
        """
        Calculate state derivative
        """
        q_nb = x[X.q_nb_0:X.q_nb_3 + 1]
        V_n = x[X.vel_N:X.vel_D + 1]
        gb_b = x[X.gyro_bias_bx:X.gyro_bias_bz + 1]
        as_b = x[X.accel_scale]

        omega_nb_b = u[U.omega_nb_bx:U.omega_nb_bz + 1]
        a_b = u[U.a_bx:U.a_bz + 1]

        # q_bn = qinverse(q_nb)

        dq_nb = 0.5*qmult(q_nb, vect2quat(omega_nb_b - gb_b))
        dV_n = rotate_vector(a_b/as_b, q_nb) - self.g_n
        dgb_b = [0, 0, 0]
        das_b = [0]
        dp_n = V_n
        dagl = 0
        dbaro_bias = 0

        return np.hstack([dq_nb, dV_n, dgb_b, das_b, dp_n,
                          dagl, dbaro_bias])

    def predict(self, t, u, dt):
        """
        Perform prediction step
        """
        # normalize quaternion
        q_nb_norm = np.linalg.norm(self.x[:4])
        if abs(q_nb_norm - 1) > 1e-3:
            self.x[:4] /= q_nb_norm
            # print('renormalizing rief:', q_nb_norm, 't:', t)

        # estimator predict
        dx = self.dynamics(self.x, u)*dt
        self.set_x(self.x + dx)

        # compute invariants
        self.J = self.compute_invariants(self.x, u)

        # linearize
        J_omega_n = self.J[:3]
        J_a_n = np.array([self.J[3:6]]).T
        I3 = np.eye(3)

        # define A matrix
        A = np.zeros((Xe.n, Xe.n))

        # derivatives of rotation error
        A[Xe.rot_bx:Xe.rot_bz + 1,
          Xe.gyro_bias_bx:Xe.gyro_bias_bz + 1] = -0.5*I3

        # derivative of velocity
        A[Xe.vel_N:Xe.vel_D + 1,
          Xe.rot_bx:Xe.rot_bz + 1] = -2*skew(J_a_n)
        A[Xe.vel_N:Xe.vel_D + 1, Xe.accel_scale] = -J_a_n.reshape(-1)

        # derivative of gyro bias
        A[Xe.gyro_bias_bx:Xe.gyro_bias_bz + 1,
          Xe.gyro_bias_bx:Xe.gyro_bias_bz + 1] = skew(J_omega_n)

        # derivative of position
        A[Xe.pos_N:Xe.pos_D + 1,
          Xe.vel_N:Xe.vel_D + 1] = I3

        # propagate covariance
        P = self.P
        Q = self.Q
        dP = (A.dot(P) + P.dot(A.T) + Q)*dt
        self.set_P(self.P + dP)

    def bound_P(self):
        """
        Constrain and bound P
        """
        for i in range(self.P.shape[0]):
            if self.P[i, i] < 0:
                print('P', i, i, '< 0, resetting')
                self.P[i, i] = 1e-6;
            for j in range(i):
                if not np.isfinite(self.P[i, j]):
                    print('P', i, j, ' is NaN, resetting')
                    self.P[i, j] = self.P0[i, j]
                    return
                # force symmetric
                self.P[j, i] = self.P[i, j]

    def bound_x(self):
        """
        Constrain and bound x
        """
        for i in range(X.gyro_bias_bx, X.gyro_bias_bz + 1):
            if self.x[i] < -0.5:
                self.x[i] = -0.5
            if self.x[i] > 0.5:
                self.x[i] = 0.5

        if self.x[X.accel_scale] > 2:
            self.x[X.accel_scale] = 2

        if self.x[X.accel_scale] < 0.5:
            self.x[X.accel_scale] = 0.5
