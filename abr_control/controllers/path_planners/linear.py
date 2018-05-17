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

    def generate_path(self, state, target, n_timesteps=200,
                 dt=0.001, plot=False):
        """ Generates a linear trajectory to the target

        Parameters
        ----------
        state : numpy.array
            the current position of the system
        target : numpy.array
            the target position
        n_timesteps : int, optional (Default: 200)
            the number of time steps to reach the target
        dt : float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        plot : boolean, optional (Default: False)
            plot the path after generating if True
        """

        self.target = target
        self.state = state
        self.dt = dt
        n_states = len(self.state)
        self.trajectory = np.zeros((n_timesteps, n_states*2))
        for ii in range(n_states):
            # calculate target states
            self.trajectory[:, ii] = np.linspace(self.state[ii],
                                                 self.target[ii],
                                                 n_timesteps)
            # calculate target velocities
            self.trajectory[:-1, ii+n_states] = (
                np.diff(self.trajectory[:, ii]) / self.dt)

        # reset trajectory index
        self.n = 0
        self.n_timesteps = n_timesteps

        if plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(np.ones((n_timesteps, n_states)) *
                              np.arange(n_timesteps)[:, None],
                     self.trajectory[:, :n_states])
            plt.gca().set_prop_cycle(None)
            plt.plot(np.ones((n_timesteps, n_states)) *
                              np.arange(n_timesteps)[:, None],
                     np.ones((n_timesteps, n_states)) * self.target, '--')
            plt.legend(['%i' % ii for ii in range(n_states)] +
                       ['%i_target' % ii for ii in range(n_states)])
            plt.title('Trajectory positions')

            plt.subplot(2, 1, 2)
            plt.plot(np.ones((n_timesteps, n_states)) *
                              np.arange(n_timesteps)[:, None],
                     self.trajectory[:, n_states:])
            plt.legend(['d%i' % ii for ii in range(n_states)])
            plt.title('Trajectory velocities')
            plt.tight_layout()

            plt.show()

    @property
    def params(self):
        params = {'source': 'linear',
                  'state': self.state,
                  'target': self.target,
                  'n_timesteps': self.n_timesteps,
                  'dt': self.dt}
        return params

    def next_target(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.trajectory[self.n]
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target
