""" Creates a trajectory from current to target orientation based on either
the timesteps (user defined profile) or n_timesteps (linear profile) passed in
"""
import numpy as np
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

    def generate_path(self, orientation, target_orientation):
        """ Generates a linear trajectory to the target

        Accepts orientations as quaternions and returns an array of orientations
        from orientation to target orientation, based on the timesteps defined
        in __init__. Orientations are returns as euler angles to match the
        format of the OSC class

        Parameters
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        """
        self.orientation_path = []

        for _ in self.timesteps:
            quat = self._step(
                orientation=orientation,
                target_orientation=target_orientation)
            self.orientation_path.append(np.copy(
                transformations.euler_from_quaternion(quat)))

        self.orientation_path = np.asarray(self.orientation_path)
        self.n = 0
        return self.orientation_path

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

        self.n = min(self.n+1, self.n_timesteps)
        return orientation


    def next(self):
        """ Returns the next step along the planned trajectory
        """
        orientation = self.orientation_path[self.n]
        self.n = min(self.n+1, self.n_timesteps)

        return orientation
