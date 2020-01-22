"""
Given two points, a path is drawn between them along the perimiter of a circle.
If the two points are of different radius from the origin, then two circles are
generated and they're paths are linearly combined to join the points. The shortest
path around the circle is taken. A linear path along z is taken.

"""
import numpy as np
import matplotlib.pyplot as plt
from .path_planner import PathPlanner


class Arc(PathPlanner):
    def __init__(self, n_timesteps=3000):
        """
        PARAMETERS
        ----------
        n_timesteps: int, Optional (Default: 3000)
            the number of steps to break the path into
        """
        self.n_timesteps = n_timesteps

    def generate_path(self, position, target_position, plot=False):
        """
        Calls the step function self.n_timestep times to pregenerate
        the entire path planner

        PARAMETERS
        ----------
        position: numpy.array
            the current position of the system
        target_position: numpy.array
            the target position
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """

        # get the angle between x cardinal dir and start pos
        phi_start = np.arctan2(position[1], position[0])
        if phi_start < 0:
            phi_start += 6.28

        # get the angle between x cardinal dir and target pos
        phi_target = np.arctan2(target_position[1], target_position[0])
        if phi_target < 0:
            phi_target += 6.28

        # if >pi diff between angles, go around the other direction
        if abs(phi_target - phi_start) > 3.14:
            if phi_target < phi_start:
                phi_target += 6.28
            elif phi_start < phi_target:
                phi_start += 6.28

        # get our number of steps along the arc
        phi = np.linspace(phi_start, phi_target, self.n_timesteps)
        # account for zero crossing
        phi = phi % 6.28

        # look at projection onto xy plane, assume z is a linear path
        r_val = lambda xyz: np.sqrt(xyz[0] ** 2 + xyz[1] ** 2 + xyz[2] ** 2)
        xval = lambda r, theta, phi: r * np.sin(theta) * np.cos(phi)
        yval = lambda r, theta, phi: r * np.sin(theta) * np.sin(phi)
        zval = lambda r, theta: r * np.cos(theta)

        r1 = r_val(position)
        theta1 = np.arccos(position[2] / r1)
        x1 = xval(r1, theta1, phi)
        y1 = yval(r1, theta1, phi)
        z1 = np.ones(self.n_timesteps) * zval(r1, theta1)

        r2 = r_val(target_position)
        theta2 = np.arccos(target_position[2] / r2)
        x2 = xval(r2, theta2, phi)
        y2 = yval(r2, theta2, phi)
        z2 = np.ones(self.n_timesteps) * zval(r2, theta2)

        linear_scaling = np.linspace(1, 0, self.n_timesteps)
        x3 = x1 * linear_scaling + x2 * linear_scaling[::-1]
        y3 = y1 * linear_scaling + y2 * linear_scaling[::-1]
        z3 = z1 * linear_scaling + z2 * linear_scaling[::-1]

        if plot:
            plt.figure()
            a = np.linspace(0, 6.48, 100)
            r = max(r1, r2)
            x = r * np.sin(a)
            y = r * np.cos(a)
            plt.plot(x, y, color="k", linestyle="--")

            plt.plot(0, 0, marker="+", c="k", label="origin")
            plt.plot(position[0], position[1], marker="*", label="start")
            plt.plot(target_position[0], target_position[1], marker="o", label="target")
            plt.plot(x1, y1, c="r", label="start arc")
            plt.plot(x2, y2, c="b", label="target arc")
            plt.plot(x3, y3, c="tab:purple", label="linearly combined")
            plt.legend()
            plt.show()

        # reset trajectory index
        self.n = 0
        self.position_path = np.vstack((np.vstack((x3, y3)), z3)).T
        self.velocity_path = np.vstack(
            (np.vstack((np.gradient(x3), np.gradient(y3))), np.gradient(z3))
        ).T

        return self.position_path, self.velocity_path
