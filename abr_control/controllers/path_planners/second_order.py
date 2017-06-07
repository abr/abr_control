import numpy as np

from .path_planner import PathPlanner


class SecondOrder(PathPlanner):
    """ Implement a trajectory controller on top of a controller

    Implements an trajectory controller on top of a given
    point to point control system. Returns a set of desired positions
    and velocities.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        super(SecondOrder, self).__init__(robot_config)

    def generate(self, state, target, n_timesteps=100,
                 plot_path=False, tau=200, k=0.005, dt=0.001):
        """ filter the target so that it doesn't jump, but moves smoothly

        Parameters
        ----------
        state : numpy.array
            the current state of the system
        target : numpy.array
            the target state
        tau: float, optional (Default:200)
            filter constant 1
        k: float, optional (Default:0.005)
            filter constant 2
        n_timesteps : int, optional (Default: 100)
            the number of time steps to reach the target
        plot_path : boolean, optional (Default: False)
            plot the path after generating if True
        """

        n_states = len(state)
        x = np.zeros((n_timesteps, n_states))
        x1 = np.zeros((n_timesteps, n_states))

        gain = 1250 / n_timesteps  # gain to converge in the desired time

        # initialize the filters to the starting position of the end effector
        x[0] = np.copy(state)

        for i in np.arange(n_timesteps - 1):
            x1[i+1] = x1[i] + (-1.0 / tau * x1[i] -k * x[i] +
                               2.0 / tau * target) * gain
            x[i+1] = x[i] + (-1.0 / tau * x[i] + k * x1[i]) * gain

        dx = np.vstack([np.diff(x, axis=0) / dt, np.zeros(x[0].shape)])
        self.trajectory = np.hstack([x, dx])

        # reset trajectory index
        self.n = 0
        self.n_timesteps = n_timesteps

        if plot_path:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(np.ones((n_timesteps, n_states)) *
                              np.arange(n_timesteps)[:, None],
                     self.trajectory[:, :n_states])
            plt.gca().set_prop_cycle(None)
            plt.plot(np.ones((n_timesteps, n_states)) *
                              np.arange(n_timesteps)[:, None],
                     np.ones((n_timesteps, n_states)) * target, '--')
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

    def next(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.trajectory[self.n]
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target
