from transforms3d.taitbryan import quat2euler
import matplotlib.pyplot as plt
import numpy as np
from util import X, Xe


class Data(object):
    """
    Data object for sim data
    """

    def __init__(self):
        self.x = []
        self.J = []
        self.K_mag = []
        self.K_gps = []
        self.K_accel = []
        self.K_lidar = []
        self.K_baro = []
        self.mag_fault = []
        self.gps_fault = []
        self.accel_fault = []
        self.lidar_fault = []
        self.baro_fault = []
        self.dx = []
        self.xh = []
        self.y = []
        self.Jh = []
        self.t = []
        self.P = []
        self.euler = None
        self.euler_est = None

    def finalize(self):
        """
        Turn lists to arrays, prepare for plotting
        """
        data = self.__dict__
        for key in data:
            data[key] = np.array(data[key])

        try:
            self.euler = np.array([quat2euler(qi)
                for qi in self.x[:, X.q_nb_0: X.q_nb_3 + 1]])
            self.euler_est = np.array([
                quat2euler(qi)
                for qi in self.xh[:, X.q_nb_0: X.q_nb_3 + 1]])
            self.agl_est = self.xh[:, X.terrain_alt] - (-self.xh[:, X.pos_D])
            self.agl = self.x[:, X.terrain_alt] - (-self.x[:, X.pos_D])

        except IndexError as e:
            print(e)

    def __repr__(self):
        return repr(self.__dict__)

    def plot_est(self, i, color, name):
        plt.plot(self.t, self.xh[:, i], color + '-', label=name + '-est')
        plt.plot(self.t, self.x[:, i], color + '--', label=name)

    def plot_est_stddev(self, i, i_error, color, name):
        plt.plot(self.t,
                 self.xh[:, i] + np.sqrt(self.P[:, i_error]), color + '-.')
        plt.plot(self.t,
                 self.xh[:, i] - np.sqrt(self.P[:, i_error]), color + '-.')

    def analysis(self):
        """
        Show plots of data

        """

        plt.rcParams['lines.linewidth'] = 2

        plt.figure(figsize=(15, 10))
        plt.title('euler angles')
        plt.plot(self.t,
                 np.rad2deg(self.euler_est[:, 2]), 'r-', label='roll-est')
        plt.plot(self.t, np.rad2deg(self.euler[:, 2]), 'r--', label='roll')
        plt.plot(self.t,
                 np.rad2deg(self.euler_est[:, 1]), 'g-', label='pitch-est')
        plt.plot(self.t, np.rad2deg(self.euler[:, 1]), 'g--', label='pitch')
        plt.plot(self.t,
                 np.rad2deg(self.euler_est[:, 0]), 'b-', label='yaw-est')
        plt.plot(self.t, np.rad2deg(self.euler[:, 0]), 'b--', label='yaw')
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.ylabel('deg')
        plt.grid()

        plt.figure(figsize=(15, 10))
        plt.subplot(221)
        plt.title('position')
        self.plot_est(X.pos_N, 'r', 'N')
        self.plot_est(X.pos_E, 'g', 'E')
        self.plot_est(X.pos_D, 'b', 'D')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.pos_N, Xe.pos_N, 'r', 'N')
        self.plot_est_stddev(X.pos_E, Xe.pos_E, 'g', 'E')
        self.plot_est_stddev(X.pos_D, Xe.pos_D, 'b', 'D')
        plt.axis(axis)
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(222)
        plt.title('velocity')
        self.plot_est(X.vel_N, 'r', 'N')
        self.plot_est(X.vel_E, 'g', 'E')
        self.plot_est(X.vel_D, 'b', 'D')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.vel_N, Xe.vel_N, 'r', 'N')
        self.plot_est_stddev(X.vel_E, Xe.vel_E, 'g', 'E')
        self.plot_est_stddev(X.vel_D, Xe.vel_D, 'b', 'D')
        plt.axis(axis)
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(223)
        plt.title('gyro bias')
        self.plot_est(X.gyro_bias_bx, 'r', 'X')
        self.plot_est(X.gyro_bias_by, 'g', 'Y')
        self.plot_est(X.gyro_bias_bz, 'b', 'Z')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.gyro_bias_bx, Xe.gyro_bias_bx, 'r', 'X')
        self.plot_est_stddev(X.gyro_bias_by, Xe.gyro_bias_by, 'g', 'Y')
        self.plot_est_stddev(X.gyro_bias_bz, Xe.gyro_bias_bz, 'b', 'Z')
        plt.axis(axis)
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(224)
        plt.title('accel scale')
        self.plot_est(X.accel_scale, 'r', '')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.accel_scale, Xe.accel_scale, 'r', '')
        plt.axis(axis)
        plt.xlabel('t, sec')
        plt.grid()

        plt.figure(figsize=(15, 10))
        plt.subplot(221)
        plt.title('agl')
        plt.plot(self.t, self.agl, '--')
        plt.plot(self.t, self.agl_est, '-')
        plt.xlabel('t, sec')
        plt.grid()

        plt.figure(figsize=(15, 10))
        plt.subplot(221)
        plt.title('terrain alt')
        self.plot_est(X.terrain_alt, 'b', '')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.terrain_alt, Xe.terrain_alt, 'r', '')
        plt.axis(axis)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(222)
        plt.title('baro bias')
        self.plot_est(X.baro_bias, 'b', '')
        axis = plt.gca().axis()
        self.plot_est_stddev(X.baro_bias, Xe.baro_bias, 'r', '')
        plt.axis(axis)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(223)
        plt.title('Invariants')
        plt.plot(self.t, self.J, '--')
        plt.gca().set_prop_cycle(None)
        plt.plot(self.t, self.Jh, '-')
        plt.xlabel('t, sec')
        plt.grid()

        plt.figure(figsize=(15, 5))
        plt.title('rotation std dev.')
        plt.plot(self.t, np.rad2deg(np.sqrt(self.P[:, Xe.rot_bx])), label='N')
        plt.plot(self.t, np.rad2deg(np.sqrt(self.P[:, Xe.rot_by])), label='E')
        plt.plot(self.t, np.rad2deg(np.sqrt(self.P[:, Xe.rot_bz])), label='D')
        plt.xlabel('t, sec')
        plt.ylabel('deg')
        plt.legend(loc='best', ncol=3)
        plt.grid()

        plt.figure(figsize=(15, 15))
        plt.subplot(321)
        plt.title('K mag')
        plt.plot(self.t, self.K_mag)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(322)
        plt.title('K gps')
        plt.plot(self.t, self.K_gps)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(323)
        plt.title('K accel')
        plt.plot(self.t, self.K_accel)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(324)
        plt.title('K_baro')
        plt.plot(self.t, self.K_baro)
        plt.xlabel('t, sec')
        plt.grid()

        plt.subplot(325)
        plt.title('K_lidar')
        plt.plot(self.t, self.K_lidar)
        plt.xlabel('t, sec')
        plt.grid()

        plt.figure(figsize=(15, 5))
        plt.title('faults')
        plt.plot(self.t, self.lidar_fault, label='lidar', alpha=0.5)
        plt.plot(self.t, self.accel_fault, label='accel', alpha=0.5)
        plt.plot(self.t, self.mag_fault, label='mag', alpha=0.5)
        plt.plot(self.t, self.gps_fault, label='gps', alpha=0.5)
        plt.plot(self.t, self.baro_fault, label='baro', alpha=0.5)
        plt.gca().set_ylim([-1, 2])
        plt.xlabel('t, sec')
        plt.legend(loc='best', ncol=3)
        plt.grid()
