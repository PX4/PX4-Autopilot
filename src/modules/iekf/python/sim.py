"""
This code investigates the use of IEKF's
"""

# pylint: disable=invalid-name, too-many-instance-attributes
# pylint: disable=unused-argument, no-member, too-many-arguments
# pylint: disable=too-many-locals, redefined-outer-name

from transforms3d.quaternions import rotate_vector, qinverse, qmult
from transforms3d.taitbryan import euler2mat
import numpy as np
from util import vect2quat, Timer, X, U, params
from data import Data


class Simulation(object):
    """
    Simulates a rigid body with fixed angular rotation
    and acceleration in body frame
    """

    def __init__(self):
        self.set_mag_field(0, 0)

    def set_mag_field(self, incl, decl):
        """
        Set the magnetic field vector
        """
        self.B_n = euler2mat(decl, -incl, 0).dot([1, 0, 0])

    @staticmethod
    def dynamics(t, x, u):
        """
        Rigid body dynamics
        """
        q_nb = x[X.q_nb_0:X.q_nb_3 + 1]
        V_n = x[X.vel_N: X.vel_D + 1]

        omega_nb_b = u[U.omega_nb_bx:U.omega_nb_bz + 1]
        a_b = u[U.a_bx:U.a_bz + 1]

        # q_bn = qinverse(q_nb)
        dq_nb = 0.5*qmult(q_nb, vect2quat(omega_nb_b))

        dV_n = rotate_vector(a_b, q_nb)
        dgb_b = [0, 0, 0]
        das_b = [0]
        dp_n = V_n
        dagl = 0
        dbaro_bias = 0

        return np.hstack([dq_nb, dV_n, dgb_b,
                          das_b, dp_n, dagl, dbaro_bias])

    def measure_accel(self, t, x, u, dt):
        """
        Accelerometer measurement model
        """
        q_nb = x[X.q_nb_0: X.q_nb_3 + 1]
        q_bn = qinverse(q_nb)
        a_s = x[X.accel_scale]
        # omega_nb_b = u[:3]
        a_b = u[U.a_bx: U.a_bz + 1]
        # gb_b = np.array(x[7:10])
        g_n = np.array([0, 0, -9.8])
        return a_s*(rotate_vector(g_n, q_bn) + a_b) + \
            params['ACC']*np.random.randn(3)/np.sqrt(dt)

    def measure_gyro(self, t, x, u, dt):
        """
        Gyro measurement model
        """
        omega_nb_b = u[:3]
        gb_b = np.array(x[7:10])
        return omega_nb_b + gb_b + \
            params['GYRO']*np.random.randn(3)/np.sqrt(dt)

    def measure_gps(self, t, x, u, dt):
        """
        GPS  measurement model
        """
        p_n = x[X.pos_N: X.pos_D + 1]
        V_n = x[X.vel_N: X.vel_D + 1]
        return np.hstack([p_n, V_n]) + \
            np.array([
                params['GPS_XY'],
                params['GPS_XY'],
                params['GPS_Z'],
                params['GPS_VXY'],
                params['GPS_VXY'],
                params['GPS_VZ'],
            ]) * np.random.randn(6)/np.sqrt(dt)

    def measure_mag(self, t, x, u, dt):
        """
        GPS  measurement model
        """
        q_nb = x[:4]
        q_bn = qinverse(q_nb)

        # Note: we don't set last component to MAG_D since
        # it would cause direction to be dominated by that component,
        # MAG_D is high to ignore the correction only
        return rotate_vector(self.B_n, q_bn) + \
            np.array([params['MAG_NE'],
                      params['MAG_NE'],
                      params['MAG_NE']]) \
            * np.random.randn(3)/np.sqrt(dt)

    def measure_baro(self, t, x, u, dt):
        """
        baro measurement model
        """
        return x[X.baro_bias] - x[X.pos_D] + \
            params['BARO_Z']*np.random.randn()/np.sqrt(dt)

    def measure_lidar(self, t, x, u, dt):
        """
        lidar measurement model
        TODO: handle angle
        """
        return -x[X.pos_D] - x[X.terrain_alt] + \
            params['LDR_Z']*np.random.randn()/np.sqrt(dt)

    def simulate(
            self, est,
            omega_nb_b,
            a_b, x0, dt, dt_est, tf):
        """
        Simulate
        """

        assert dt < dt_est

        data = Data()
        x = x0
        t = 0

        gps_timer = Timer(t, 1.0/10)
        mag_timer = Timer(t, 1.0/50)
        accel_timer = Timer(t, 1.0/50)
        baro_timer = Timer(t, 1.0/10)
        lidar_timer = Timer(t, 1.0/10)

        u = np.hstack([omega_nb_b, a_b])

        while t + dt < tf:

            # normalize quaternion
            q_nb_norm = np.linalg.norm(x[:4])
            if abs(q_nb_norm - 1) > 1e-3:
                x[:4] /= q_nb_norm
                # print('renormalizing sim', q_nb_norm)

            # sim propagate
            x = x + self.dynamics(t, x, u)*dt
            y_gyro = self.measure_gyro(t, x, u, dt)
            y_accel = self.measure_accel(t, x, u, dt)

            # estimator predict
            u_meas = np.hstack([y_gyro, y_accel])
            est.predict(t, u_meas, dt)

            # compute invariants
            J = est.compute_invariants(x, u)
            Jh = est.compute_invariants(est.x, u)

            # estimator correct
            if gps_timer.ready(t):
                y_gps = self.measure_gps(t, x, u, gps_timer.period)
                est.correct_gps(y_gps, gps_timer.period)
            if mag_timer.ready(t):
                y_mag = self.measure_mag(t, x, u, mag_timer.period)
                est.correct_mag(y_mag, mag_timer.period)
            if baro_timer.ready(t):
                y_baro = self.measure_baro(t, x, u, baro_timer.period)
                est.correct_baro(y_baro, baro_timer.period)
            if lidar_timer.ready(t):
                y_lidar = self.measure_lidar(t, x, u, lidar_timer.period)
                est.correct_lidar(y_lidar, lidar_timer.period)
            if accel_timer.ready(t):
                est.correct_accel(y_accel, accel_timer.period)

            data.x += [x]
            data.K_gps += [est.K_gps.reshape(-1)]
            data.K_mag += [est.K_mag.reshape(-1)]
            data.K_accel += [est.K_accel.reshape(-1)]
            data.K_baro += [est.K_baro.reshape(-1)]
            data.K_lidar += [est.K_lidar.reshape(-1)]
            data.accel_fault += [est.accel_fault]
            data.baro_fault += [est.baro_fault]
            data.mag_fault += [est.mag_fault]
            data.gps_fault += [est.gps_fault]
            data.lidar_fault += [est.lidar_fault]
            data.J += [J]
            data.Jh += [Jh]
            data.xh += [est.x]
            data.y += [y_accel]
            data.t += [t]
            data.P += [np.diag(est.P).reshape(-1)]

            t += dt

        data.finalize()
        return data
