import numpy as np

class Linear():
    """ Creates a linear trajectory from current to target state
    """

    def generate_path(self, state, target, n_timesteps=None,
                      dx=None, dt=0.001, plot=False):
        """ Generates a linear trajectory to the target

        Parameters
        ----------
        state: numpy.array
            the current position of the system
        target: numpy.array
            the target position
        n_timesteps: int, optional (Default: 200)
            the number of time steps to reach the target
            cannot be specified at the same time as dx
        dx: float, optional (Default: None)
            the distance to move each timestep
            cannot be specified at same time as n_timesteps
        dt: float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """
        assert n_timesteps is None or dx is None

        n_states = len(state)

        if n_timesteps is not None:
            self.trajectory = np.zeros((n_timesteps, n_states*2))
            for ii in range(n_states):
                # calculate target states
                self.trajectory[:, ii] = np.linspace(
                    state[ii], target[ii], n_timesteps)
        else:
            distance = (target - state)
            norm_distance = np.linalg.norm(distance)
            step = distance / norm_distance * dx
            n_timesteps = int(np.ceil(norm_distance / dx))
            self.trajectory = np.zeros((n_timesteps, n_states*2))

            for ii in range(n_states):
                # calculate target states
                if abs(step[ii]) > 1e-5:
                    self.trajectory[:, ii] = np.arange(
                        state[ii], target[ii], step[ii])
                    # calculate target velocities
                    self.trajectory[:-1, ii+n_states] = (
                        np.diff(self.trajectory[:, ii]) / dt)

        # reset trajectory index
        self.n = 0
        self.n_timesteps = self.trajectory.shape[0]

        if plot:
            n_timesteps = self.trajectory.shape[0]
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
