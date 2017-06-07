import numpy as np

from .path_planner import PathPlanner


class Linear(PathPlanner):
    """ Creates a linear trajectory from current to target state

    Parameters
    ----------
    robot_config : class instance
        contains all relevant information about the arm
        such as: number of joints, number of links, mass information etc.
    """

    def __init__(self, robot_config):
        super(Linear, self).__init__(robot_config)

    def generate(self, state, target, n_timesteps=200, dt=0.001):
        """ Generates a linear trajectory to the target

        Parameters
        ----------
        state : numpy.array
            the current position of the system [meters]
        target : numpy.array
            the target position [radians]
        n_timesteps : int, optional (Default: 200)
            the number of time steps to reach the target
        dt : float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        """

        n_states = len(state)
        self.trajectory = np.zeros((n_timesteps, n_states*2))
        for ii in range(n_states):
            # calculate target states
            self.trajectory[:, ii] = np.linspace(state[ii],
                                                 target[ii],
                                                 n_timesteps)
            # calculate target velocities
            self.trajectory[:-1, ii+n_states] = (
                np.diff(self.trajectory[:, ii]) / 0.001)

        # reset trajectory index
        self.n = 0
        self.n_timesteps = n_timesteps

    def next(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.trajectory[self.n]
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target
