import numpy as np

from . import path_planner

class Linear(path_planner.PathPlanner):
    """ Implements an trajectory controller over on top of a given
    point to point control system. Returns a set of desired positions
    and velocities.
    """

    def __init__(self, robot_config):
        super(StraightLine, self).__init__(robot_config)

    def generate(self, state, target, n_timesteps=200, dt=0.001):
        """ Generates a trajectory to move from the current position
        to the target position in a straight line.

        state np.array: the current position of the system
        target np.array: the target position
        n_timesteps int: the number of time steps to reach the target
        dt float: the time step for calculating desired velocities
        """

        n_states = len(state)
        self.trajectory = np.zeros((n_timesteps, n_states*2))
        for ii in range(n_states):
            # calculate target states
            self.trajectory[:, ii] = np.linspace(xyz[ii],
                                                 target_state[ii],
                                                 n_timesteps)
            # calculate target velocities
            self.trajectory[:-1, ii+n_states] = (
                np.diff(self.trajectory[:, ii]) / 0.001)

        # reset trajectory index
        self.n = 0

    def next(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target_state = (self.trajectory[self.n]
                             if self.n < n_timesteps else self.target_state)
        self.n += 1

        return self.target_state
