import numpy as np

class Linear():
    """ Creates a linear trajectory from current to target state

    Parameters
    ----------
    """
    def generate_path(self, state, target, n_timesteps=None,
                      dx=None, dt=0.001, plot=False):
        """ Generates a linear trajectory to the target

        Parameters
        ----------
        state : numpy.array
            the current position of the system
        target : numpy.array
            the target position
        n_timesteps : int, optional (Default: 200)
            the number of time steps to reach the target
            cannot be specified at the same time as dx
        dx : float, optional (Default: None)
            the distance to move each timestep
            cannot be specified at same time as n_timesteps
        dt : float, optional (Default: 0.001)
            the time step for calculating desired velocities [seconds]
        plot : boolean, optional (Default: False)
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
            self.trajectory = []
            step = (target - state)
            step = step / np.linalg.norm(step) * dx
            zero_dims = []
            for ii in range(n_states):
                if step[ii] == 0:
                    zero_dims.append(ii)
                else:
                    # calculate target states
                    self.trajectory.append(np.arange(
                        state[ii], target[ii], step[ii]))
            if len(zero_dims) > 0:
                zero_row = np.squeeze(np.zeros(len(self.trajectory[0])))
                tmp_trajectory = []
                for index in range(n_states):
                    if index in zero_dims:
                        tmp_trajectory.append(zero_row)
                    else:
                        tmp_trajectory.append(self.trajectory[index])
                self.trajectory = tmp_trajectory

            zeros = np.zeros(self.trajectory[ii].shape)
            for ii in range(6 - len(self.trajectory)):
                self.trajectory.append(zeros)
            self.trajectory = np.vstack(self.trajectory).T

        for ii in range(n_states):
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
