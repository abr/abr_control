import numpy as np
import matplotlib
matplotlib.use("TKAgg")

from .path_planner import PathPlanner


class SecondOrder(PathPlanner):
    """ Implement a trajectory controller on top of a controller

    Implements an trajectory controller on top of a given
    point to point control system. Returns a set of desired positions
    and velocities.
    Implements the second order filter from
    www.mathworks.com/help/physmod/sps/powersys/ref/secondorderfilter.html

    returns the next state in the path in the form
    np.hstack([posx, posy, posz, velx, vely, velz])

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    n_timesteps : int, optional (Default: 100)
        the number of time steps to reach the target
    dt : float, optional (Default: 0.001)
        the loop speed [seconds]
    zeta  float, optional (Default: 2.0)
        the damping ratio
    w : float, optional (Default: 1e-4)
        the natural frequency
    """

    def __init__(self, robot_config, n_timesteps=100, dt=0.001, zeta=2.0,
                 w=1e4):
        super(SecondOrder, self).__init__(robot_config)
        self.n_timesteps = n_timesteps
        self.dt = dt
        self.zeta = zeta
        self.w = w/n_timesteps # gain to converge in the desired time

    def step(self, state, target):
        """ Calculates the next state given the current state and
        system dynamics' parameters.

        Parameters
        ----------
        state : numpy.array
            the current position and velocity of the system
            use the format state=np.array([posx, posy, posz, velx, vely, velz])
        target : numpy.array
            the target position of the system
        """
        y = state[:3]
        dy = state[3:]
        ddy = self.w**2 * target - dy * self.zeta * self.w - y * self.w**2
        state += np.hstack((dy, ddy)) * self.dt
        return state

    def generate_path(self, state, target, plot=False):
        """ Filter the target so that it doesn't jump, but moves smoothly

        Parameters
        ----------
        state : numpy.array
            the current position and velocity of the system
            use the format state=np.array([posx, posy, posz, velx, vely, velz])
        target : numpy.array
            the target state
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """

        n_states = int(len(state)/2)

        self.trajectory = []
        for ii in range(self.n_timesteps):
            self.trajectory.append(np.copy(state))
            state = self.step(state=state, target=target)
        self.trajectory = np.array(self.trajectory)

        # reset trajectory index
        self.n = 0

        if plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(np.ones((self.n_timesteps, n_states)) *
                              np.arange(self.n_timesteps)[:, None],
                     self.trajectory[:, :n_states])
            plt.gca().set_prop_cycle(None)
            plt.plot(np.ones((self.n_timesteps, n_states)) *
                              np.arange(self.n_timesteps)[:, None],
                     np.ones((self.n_timesteps, n_states)) * target, '--')
            plt.legend(['%i' % ii for ii in range(n_states)] +
                       ['%i_target' % ii for ii in range(n_states)])
            plt.title('Trajectory positions')

            plt.subplot(2, 1, 2)
            plt.plot(np.ones((self.n_timesteps, n_states)) *
                              np.arange(self.n_timesteps)[:, None],
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
