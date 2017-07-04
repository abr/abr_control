import numpy as np

from .path_planner import PathPlanner


class SecondOrder(PathPlanner):
    """ Implement a trajectory controller on top of a controller

    Implements an trajectory controller on top of a given
    point to point control system. Returns a set of desired positions
    and velocities.
    Implements the second order filter from
    www.mathworks.com/help/physmod/sps/powersys/ref/secondorderfilter.html

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        super(SecondOrder, self).__init__(robot_config)

    def step(self, y, dy, target, w, zeta):
        """ Calculates the next state given the current state and
        system dynamics' parameters.

        Parameters
        ----------
        y : numpy.array
            the current position of the system
        dy : numpy.array
            the current velocity of the system
        target : numpy.array
            the target position of the system
        w : float
            the natural frequency
        zeta : float
            the damping ratio
        """
        ddy = w**2 * target - dy * zeta * w - y * w**2
        return dy, ddy

    def generate_path(self, state, target, n_timesteps=100,
                      plot=False, zeta=2.0, dt=0.001):
        """ Filter the target so that it doesn't jump, but moves smoothly

        Parameters
        ----------
        state : numpy.array
            the current position of the system
        target : numpy.array
            the target state
        n_timesteps : int, optional (Default: 100)
            the number of time steps to reach the target
        zeta  float, optional (Default: 2.0)
            the damping ratio
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """

        n_states = len(state)

        w = 1e4 / n_timesteps # gain to converge in the desired time

        self.trajectory = []
        y = np.hstack([state, np.zeros(n_states)])
        for ii in range(n_timesteps):
            self.trajectory.append(np.copy(y))
            y += np.hstack(self.step(
                y[:n_states], y[n_states:], target, w, zeta)
                ) * dt
        self.trajectory = np.array(self.trajectory)

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

    def next_target(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.trajectory[self.n]
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target
