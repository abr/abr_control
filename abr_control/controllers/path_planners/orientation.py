""" Creates a trajectory from current to target orientation based on either
the timesteps (user defined profile) or n_timesteps (linear profile) passed in
"""
import numpy as np

import matplotlib.pyplot as plt

from abr_control.utils import transformations


class Orientation():
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
    def __init__(self, n_timesteps=None, timesteps=None):
        assert n_timesteps is None or timesteps is None

        if n_timesteps is not None:
            self.n_timesteps = n_timesteps
            self.timesteps = np.linspace(0, 1, self.n_timesteps)

        elif timesteps is not None:
            self.timesteps = timesteps
            self.n_timesteps = len(timesteps)

        self.n = 0


    def generate_path(self, orientation, target_orientation, plot=False):
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
        """
        # stores the target Euler angles of the trajectory
        self.orientation = np.zeros((self.n_timesteps, 3))
        self.target_angles = transformations.euler_from_quaternion(
            target_orientation, axes='rxyz')

        for ii in range(self.n_timesteps):
            quat = self._step(
                orientation=orientation,
                target_orientation=target_orientation)
            self.orientation[ii] = transformations.euler_from_quaternion(
                quat, axes='rxyz')

        self.n = 0
        if plot:
            self._plot()

        return self.orientation


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
            quat0=orientation, quat1=target_orientation,
            fraction=self.timesteps[self.n])

        self.n = min(self.n+1, self.n_timesteps-1)
        return orientation


    def next(self):
        """ Returns the next step along the planned trajectory

        NOTE: only orientation is returned, no target velocity
        """
        orientation = self.orientation[self.n]
        self.n = min(self.n+1, self.n_timesteps-1)

        return orientation


    def _plot(self):
        """ Plot the generated trajectory
        """
        plt.figure()
        for ii, path in enumerate(self.orientation.T):
            plt.plot(path, lw=2, label='Trajectory')
            plt.plot(
                np.ones(path.shape) * self.target_angles[ii], '--',
                lw=2, label='Target angles')
        plt.xlabel('Radians')
        plt.ylabel('Time step')
        plt.legend()
        plt.show()
