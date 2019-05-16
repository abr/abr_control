import numpy as np
import scipy

class PathPlanner:

    def generate_path(self):
        NotImplementedError # pylint: disable=W0104


    def _step(self):
        NotImplementedError # pylint: disable=W0104


    def convert_to_time(self, pregenerated_path, time_limit):
        """
        Accepts a pregenerated path from current state to target and
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
        from .orientation import Orientation
        error = []
        dist = np.sqrt(np.sum((self.position[-1] - self.position[0])**2))
        for ee in self.position:
            error.append(np.sqrt(np.sum((self.position[-1] - ee)**2)))
        error /= dist
        error = 1 - error

        self.orientation = Orientation(timesteps=error)
        self.orientation.generate_path(
            orientation=orientation, target_orientation=target_orientation)

        if plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(error)
            plt.xlabel('time steps')
            plt.ylabel('trajectory step')
            plt.show()

    #NOTE: do we need this check? should it be in the generate_path func?
    # def check_convergence(self, target_pos, position):
    #     # check if we are within 1cm of our target by the end of our path
    #     dist = np.sqrt(np.sum((target_pos - position[-1])**2))
    #     if dist > 0.001:
    #         self.n_timesteps += 10
    #         return False
    #     else:
    #         return True
