import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import math


class LoopTrajectory:
    def __init__(self, viapoints, t_tot):
        """
        Initialize with an array of viapoints. Viapoints should be a Nx2 array.
        """
        self.viapoints = np.array(viapoints)
        # if the last and first viapoints does not match, append
        if not np.allclose(self.viapoints[0], self.viapoints[-1]):
            self.viapoints = np.vstack([self.viapoints, self.viapoints[0]])
        self.t_tot = t_tot
        self.compute_spline(t_tot)

    def compute_spline(self, t_tot):
        """
        Compute cubic spline for the given viapoints.
        """
        t = np.linspace(0, t_tot, len(self.viapoints))
        self.spline_x = CubicSpline(
            t, self.viapoints[:, 0], bc_type='periodic')
        self.spline_y = CubicSpline(
            t, self.viapoints[:, 1], bc_type='periodic')

    def eval_trajectory(self, t):
        """
        Compute x, y, omega, x_dot, y_dot, omega_dot as a function of t.
        """
        x = self.spline_x(t)
        y = self.spline_y(t)
        x_dot = self.spline_x(t, 1)
        y_dot = self.spline_y(t, 1)

        theta = np.arctan2(y_dot.item(0), x_dot.item(0))

        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])

        b_vel_xy = (w_R_b.T).dot(np.array([x_dot, y_dot]))
        # v = np.linalg.norm(b_vel_xy)

        # only x because is the tangential part, y is the radial and is not 0, we assume unycycle so no radial part
        v = b_vel_xy[0]

        x_ddot = self.spline_x(t, 2)
        y_ddot = self.spline_y(t, 2)

        omega = (x_dot * y_ddot - y_dot * x_ddot) / (x_dot**2 + y_dot**2)

        b_vel_dot_xy = (w_R_b.T).dot(np.array([x_ddot, y_ddot]))
        # v_dot = np.linalg.norm(b_vel_dot_xy)
        v_dot = b_vel_dot_xy[0]

        omega_dot = 0.

        return np.squeeze(x), np.squeeze(y), np.squeeze(theta), np.squeeze(v), np.squeeze(omega), np.squeeze(v_dot),np.squeeze(omega_dot)

    def plot_trajectory(self):
        """
        Plot the trajectory for visualization.
        """
        t = np.linspace(0, self.t_tot, 100)
        x, y, _, _, _, _, _ = self.eval_trajectory(t)

        plt.figure()
        plt.plot(x, y, label='Trajectory')
        plt.plot(self.viapoints[:, 0],
                 self.viapoints[:, 1], 'ro', label='Points')
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Closed Loop Trajectory')
        plt.grid()
        plt.show()
