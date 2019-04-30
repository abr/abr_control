import numpy as np


class SecondOrder:
    """ Implement a trajectory controller on top of a controller

    Implements a second order filter for path generation.
    Returns a set of target positions and velocities.
    Implements the second order filter from
    www.mathworks.com/help/physmod/sps/powersys/ref/secondorderfilter.html

    returns target in form [positions, velocities]

    Parameters
    ----------
    n_timesteps: int, optional (Default: 100)
        the number of time steps to reach the target
    dt: float, optional (Default: 0.001)
        the loop speed [seconds]
    zeta: float, optional (Default: 2.0)
        the damping ratio
    w: float, optional (Default: 1e-4)
        the natural frequency
    threshold: float, optional (Default: 0.02)
        within this threshold distance to target position reduce the
        filtering effects to improve convergence in practice
    """

    def __init__(self, n_timesteps=100, dt=0.001,
                 zeta=2.0, w=1e4, threshold=0.02):

        self.n_timesteps = n_timesteps
        self.dt = dt
        self.zeta = zeta
        self.w = w/n_timesteps # gain to converge in the desired time
        self.threshold = threshold


    def step(self, state, target_pos, dt=0.001):
        """ Calculates the next state given the current state and
        system dynamics' parameters.

        Parameters
        ----------
        state : numpy.array
            the current position and velocity of the system
            use the format state=np.array([positions, velocities])
        target_pos : numpy.array
            the target position of the system
        dt : float
            the time step to move forward
        """
        n_states = int(len(state)/2)
        y = state[:n_states]
        dy = state[n_states:]

        w = self.w
        if np.linalg.norm(y - target_pos) < self.threshold:
            # if within a threshold distance, reduce the filter effect
            # NOTE: this is a ad-hoc method of improving performance at
            # short distances
            w *= 3

        ddy = w**2 * target_pos - dy * self.zeta * w - y * w**2
        dy = dy + ddy * dt
        y = y + dy * dt
        return np.hstack((y, dy))


    def generate_path(self, state, target_pos, plot=False):
        """ Filter the target so that it doesn't jump, but moves smoothly

        Parameters
        ----------
        state : numpy.array
            the current position and velocity of the system
            use the format state=np.array([positions, velocities])
        target_pos : numpy.array
            the target position
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """

        n_states = int(len(state)/2)

        self.trajectory = []
        for _ in range(self.n_timesteps):
            self.trajectory.append(np.copy(state))
            state = self.step(state, target_pos, dt=self.dt)
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
                     np.ones((self.n_timesteps, n_states)) * target_pos, '--')
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

    def generate_path_function(self, state, target, time_limit,
                               target_vel=False):
        self.n_timesteps = 3000
        self.generate_path(state=state, target_pos=target)
        dist = np.sqrt(np.sum((target[:3] - self.trajectory[-1][:3])**2))
        print('Checking that path reaches target within %.2f seconds'
              % time_limit)
        while dist > 0.01:
            self.n_timesteps += 10
            self.generate_path(state=state, target_pos=target)
            dist = np.sqrt(np.sum((target[:3] - self.trajectory[-1][:3])**2))
            print(self.n_timesteps)
        import scipy.interpolate
        times = np.linspace(0, time_limit, self.n_timesteps)
        x = scipy.interpolate.interp1d(times, self.trajectory[:, 0])
        y = scipy.interpolate.interp1d(times, self.trajectory[:, 1])
        z = scipy.interpolate.interp1d(times, self.trajectory[:, 2])
        if target_vel:
            dx = scipy.interpolate.interp1d(
                times, np.gradient(self.trajectory[:, 0]))
            dy = scipy.interpolate.interp1d(
                times, np.gradient(self.trajectory[:, 1]))
            dz = scipy.interpolate.interp1d(
                times, np.gradient(self.trajectory[:, 2]))
            self.path_func = [x, y, z, dx, dy, dz]
        else:
            self.path_func = [x, y, z]

    def next_timestep(self, t):
        target = []
        for dim in self.path_func:
            target.append(dim(t))
        return target
