""" Creates a trajectory from current to target orientation based on either
the timesteps (user defined profile) or n_timesteps (linear profile) passed in
"""
import numpy as np

import matplotlib.pyplot as plt

from abr_control.utils import transformations
from .path_planner import PathPlanner


class Orientation(PathPlanner):
    """
    PARAMETERS
    ----------
    n_timesteps: int, optional (Default: 200)
        the number of time steps to reach the target
        cannot be specified at the same time as timesteps
    timesteps: array of floats
        the cumulative step size to take from 0 (start orientation) to
        1 (target orientation)
    """

    def __init__(self, n_timesteps=None, timesteps=None, axes="rxyz"):
        # assert n_timesteps is None or timesteps is None
        self.axes = axes

        if n_timesteps is not None:
            self.n_timesteps = n_timesteps
            self.timesteps = np.linspace(0, 1, self.n_timesteps)

        elif timesteps is not None:
            self.timesteps = timesteps
            self.n_timesteps = len(timesteps)

        self.n = 0

    def generate_path(self, orientation, target_orientation, dr=None, plot=False):
        """ Generates a linear trajectory to the target

        Accepts orientations as quaternions and returns an array of orientations
        from orientation to target orientation, based on the timesteps defined
        in __init__. Orientations are returns as euler angles to match the
        format of the OSC class

        NOTE: no velocity trajectory is calculated at the moment

        Parameters
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        dr: float, Optional (Default: None)
            if not None the path to target is broken up into n_timesteps segments.
            Otherwise the number of timesteps are determined based on the set step
            size in radians.
        """
        if len(orientation) == 3:
            raise ValueError(
                "\n----------------------------------------------\n"
                + "A quaternion is required as input for the orientation "
                + "path planner. To convert your "
                + "Euler angles into a quaternion run...\n\n"
                + "from abr_control.utils import transformations\n"
                + "quaternion = transformation.quaternion_from_euler(a, b, g)\n"
                + "----------------------------------------------"
            )

        self.target_angles = transformations.euler_from_quaternion(
            target_orientation, axes=self.axes
        )

        if dr is not None:
            # angle between two quaternions
            # "https://www.researchgate.net/post"
            # + "/How_do_I_calculate_the_smallest_angle_between_two_quaternions"
            # answer by luiz alberto radavelli
            angle_diff = 2 * np.arccos(
                np.dot(target_orientation, orientation)
                / (np.linalg.norm(orientation) * np.linalg.norm(target_orientation))
            )

            if angle_diff > np.pi:
                min_angle_diff = 2 * np.pi - angle_diff
            else:
                min_angle_diff = angle_diff

            self.n_timesteps = int(min_angle_diff / dr)

            print(
                "%i steps to cover %f rad in %f sized steps"
                % (self.n_timesteps, angle_diff, dr)
            )
            self.timesteps = np.linspace(0, 1, self.n_timesteps)

        # stores the target Euler angles of the trajectory
        self.orientation_path = np.zeros((self.n_timesteps, 3))

        for ii in range(self.n_timesteps):
            quat = self._step(
                orientation=orientation, target_orientation=target_orientation
            )
            self.orientation_path[ii] = transformations.euler_from_quaternion(
                quat, axes=self.axes
            )
        if self.n_timesteps == 0:
            print("with the set step size, we reach the target in 1 step")
            self.orientation_path = np.array(
                [
                    transformations.euler_from_quaternion(
                        target_orientation, axes=self.axes
                    )
                ]
            )

        self.n = 0

        if plot:
            self._plot()

        return self.orientation_path

    def _const_step(self, orientation, target_orientation):
        """ Calculates the next step along the planned trajectory

        PARAMETERS
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        """
        orientation = transformations.quaternion_slerp(
            quat0=orientation, quat1=target_orientation, fraction=self.timesteps[self.n]
        )

        self.n = min(self.n + 1, self.n_timesteps - 1)
        return orientation

    def _step(self, orientation, target_orientation):
        """ Calculates the next step along the planned trajectory

        PARAMETERS
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        """
        orientation = transformations.quaternion_slerp(
            quat0=orientation, quat1=target_orientation, fraction=self.timesteps[self.n]
        )

        self.n = min(self.n + 1, self.n_timesteps - 1)
        return orientation

    def next(self):
        """ Returns the next step along the planned trajectory

        NOTE: only orientation is returned, no target velocity
        """
        orientation = self.orientation_path[self.n]
        self.n = min(self.n + 1, self.n_timesteps - 1)

        return orientation

    def match_position_path(
        self, orientation, target_orientation, position_path, plot=False
    ):
        """ Generates orientation trajectory with the same profile as the path
        generated for position

        Ex: if a second order filter is applied to the trajectory, the same will
        be applied to the orientation trajectory

        PARAMETERS
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        plot: boolean, Optional (Default: False)
            True to plot the profile of the steps taken from start to target
            orientation
        """

        error = []
        dist = np.sqrt(np.sum((position_path[-1] - position_path[0]) ** 2))
        for ee in position_path:
            error.append(np.sqrt(np.sum((position_path[-1] - ee) ** 2)))
        error /= dist
        error = 1 - error

        self.timesteps = error
        self.n_timesteps = len(self.timesteps)
        self.orientation_path = self.generate_path(
            orientation=orientation, target_orientation=target_orientation, plot=plot
        )

        return self.orientation_path

    def _plot(self):
        """ Plot the generated trajectory
        """
        plt.figure()
        for ii, path in enumerate(self.orientation_path.T):
            plt.plot(path, lw=2, label="Trajectory")
            plt.plot(
                np.ones(path.shape) * self.target_angles[ii],
                "--",
                lw=2,
                label="Target angles",
            )
        plt.xlabel("Radians")
        plt.ylabel("Time step")
        plt.legend()
        plt.show()
