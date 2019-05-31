import numpy as np

import matplotlib.pyplot as plt

import scipy.interpolate

from .orientation import Orientation


class PathPlanner:
    """ Base class for path planners.
    """

    def generate_path(self):
        """ This function generates the trajectory to follow, storing the
        results and returning self.position and self.velocity
        """
        raise NotImplementedError


    def convert_to_time(self, pregenerated_path, time_limit):
        """ Accepts a pregenerated path from current state to target and
        interpolates with respect to the time_limit. The function can
        then be stepped through to reach a target within the specified time.

        PARAMETERS
        ----------
        pregenerated_path: numpy.array
            The output from a subclasses generate_path() function
        time_limit: float
            the desired time to go from state to target [seconds]
        """

        n_states = np.asarray(pregenerated_path).shape[1]

        # interpolate the function to the specified time_limit
        times = np.linspace(0, time_limit, self.n_timesteps)
        self.path = []
        for dim in range(n_states):
            self.path.append(scipy.interpolate.interp1d(
                times, pregenerated_path[:, dim]))

        return self.path


    def generate_orientation_path(self, orientation, target_orientation,
                                  plot=False):
        """ Generates orientation trajectory with the same profile as the path
        generated for position

        Ex: if a second order filter is applied to the trajectory, the same will
        be applied to the orientation trajectory

        PARAMETERS
        ----------
        orientation: list of 4 floats
            the starting orientation as a quaternion
        target_orientation: list of 4 floats
            the target orientation as a quaternion
        plot: boolean, Optional (Default: False)
            True to plot the profile of the steps taken from start to target
            orientation
        """

        error = []
        dist = np.sqrt(np.sum((self.position[-1] - self.position[0])**2))
        for ee in self.position:
            error.append(np.sqrt(np.sum((self.position[-1] - ee)**2)))
        error /= dist
        error = 1 - error

        orientation_path_planner = Orientation(timesteps=error)
        orientation_path = orientation_path_planner.generate_path(
            orientation=orientation, target_orientation=target_orientation,
            plot=plot)

        return orientation_path, orientation_path_planner


    #NOTE: do we need this check? should it be in the generate_path func?
    # def check_convergence(self, target_pos, position):
    #     # check if we are within 1cm of our target by the end of our path
    #     dist = np.sqrt(np.sum((target_pos - position[-1])**2))
    #     if dist > 0.001:
    #         self.n_timesteps += 10
    #         return False
    #     else:
    #         return True


    def next(self):
        """ Returns the next target from the generated path
        """
        position = self.position[self.n]  # pylint: disable=E0203
        velocity = self.velocity[self.n]  # pylint: disable=E0203
        self.n = min(self.n+1, self.n_timesteps - 1)

        return position, velocity


    def _plot(self, target_pos):
        """ Plot the generated trajectory

        PARAMETERS
        ----------
        target_pos: np.array
            the final target position the trajectory is moving to
        """
        n_states = len(self.position[0])

        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(np.ones((self.n_timesteps, n_states))
                 * np.arange(self.n_timesteps)[:, None],
                 self.position)
        plt.gca().set_prop_cycle(None)
        plt.plot(
            np.ones((self.n_timesteps, n_states))
            * np.arange(self.n_timesteps)[:, None],
            np.ones((self.n_timesteps, n_states)) * target_pos,
            '--')
        plt.legend(['%i' % ii for ii in range(n_states)]
                   + ['%i_target' % ii for ii in range(n_states)])
        plt.title('Trajectory positions')

        plt.subplot(2, 1, 2)
        plt.plot(np.ones((self.n_timesteps, n_states))
                 * np.arange(self.n_timesteps)[:, None], self.velocity)
        plt.legend(['d%i' % ii for ii in range(n_states)])
        plt.title('Trajectory velocities')
        plt.tight_layout()

        plt.show()
