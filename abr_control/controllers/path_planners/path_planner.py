import numpy as np

import matplotlib.pyplot as plt

import scipy.interpolate


class PathPlanner:
    """ Base class for path planners.
    """

    def generate_path(self):
        """ This function generates the trajectory to follow, storing the
        results and returning self.position_path and self.velocity_path
        """
        raise NotImplementedError

    def convert_to_time(self, path, time_length):
        """ Accepts a pregenerated path from current state to target and
        interpolates with respect to the time_limit. The function can
        then be stepped through to reach a target within the specified time.

        PARAMETERS
        ----------
        path: numpy.array
            The output from a subclasses generate_path() function
        time_length: float
            the desired time to go from state to target [seconds]
        """

        n_states = np.asarray(path).shape[1]

        # interpolate the function to the specified time_limit
        times = np.linspace(0, time_length, self.n_timesteps)
        path_func = []
        for dim in range(n_states):
            path_func.append(
                scipy.interpolate.interp1d(times, np.asarray(path)[:, dim])
            )

        return path_func

    def next(self):
        """ Returns the next target from the generated path
        """
        position = self.position_path[self.n]  # pylint: disable=E0203
        velocity = self.velocity_path[self.n]  # pylint: disable=E0203
        self.n = min(self.n + 1, self.n_timesteps - 1)

        return position, velocity

    def _plot(self, target_position):
        """ Plot the generated trajectory

        PARAMETERS
        ----------
        target_position: np.array
            the final target position the trajectory is moving to
        """
        n_states = len(self.position_path[0])

        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(
            np.ones((self.n_timesteps, n_states))
            * np.arange(self.n_timesteps)[:, None],
            self.position_path,
        )
        plt.gca().set_prop_cycle(None)
        plt.plot(
            np.ones((self.n_timesteps, n_states))
            * np.arange(self.n_timesteps)[:, None],
            np.ones((self.n_timesteps, n_states)) * target_position,
            "--",
        )
        plt.legend(
            ["%i" % ii for ii in range(n_states)]
            + ["%i_target" % ii for ii in range(n_states)]
        )
        plt.title("Trajectory positions")

        plt.subplot(2, 1, 2)
        plt.plot(
            np.ones((self.n_timesteps, n_states))
            * np.arange(self.n_timesteps)[:, None],
            self.velocity_path,
        )
        plt.legend(["d%i" % ii for ii in range(n_states)])
        plt.title("Trajectory velocities")
        plt.tight_layout()

        plt.show()
