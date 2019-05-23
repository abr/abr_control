import numpy as np

from .path_planner import PathPlanner


class Linear(PathPlanner):
    """ Creates a linear trajectory from current to target state

    PARAMETERS
    ----------
    n_timesteps: int, optional (Default: 200)
        the number of time steps to reach the target
        cannot be specified at the same time as dx
    dx: float, optional (Default: None)
        the distance to move each timestep
        cannot be specified at same time as n_timesteps
    dt: float, optional (Default: 0.001)
        the time step for calculating desired velocities [seconds]
    """
    def __init__(self, n_timesteps=None, dx=None, dt=0.001):
        assert ((n_timesteps is None and dx is not None) or
                (dx is None and n_timesteps is not None))

        self.n_timesteps = n_timesteps
        self.dx = dx
        self.dt = dt

    def generate_path(self, position, target_pos, plot=False):
        """ Generates a linear trajectory to the target

        Parameters
        ----------
        position : numpy.array
            the current position of the system
        target_pos : numpy.array
            the target position
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """
        n_states = len(position)

        if self.dx is None:
            self.position = np.zeros((self.n_timesteps, n_states))
            self.velocity = np.zeros((self.n_timesteps, n_states))
            for ii in range(n_states):
                # calculate target states
                self.position[:, ii] = np.linspace(
                    position[ii], target_pos[ii], self.n_timesteps)
        else:
            distance = (target_pos - position)
            norm_distance = np.linalg.norm(distance)
            step = distance / norm_distance * self.dx

            self.n_timesteps = int(np.ceil(norm_distance / self.dx))

            self.position = np.zeros((self.n_timesteps, n_states))
            self.velocity = np.zeros((self.n_timesteps, n_states))

            for ii in range(n_states):
                # calculate target states
                if abs(step[ii]) > 1e-5:
                    self.position[:, ii] = np.arange(
                        position[ii], target_pos[ii], step[ii])

        for ii in range(n_states):
            # calculate target velocities
            self.velocity[:-1, ii] = (
                np.diff(self.position[:, ii]) / self.dt)

        # reset trajectory index
        self.n = 0
        self.n_timesteps = self.position.shape[0]

        if plot:
            self._plot(target_pos)

        return self.position, self.velocity
