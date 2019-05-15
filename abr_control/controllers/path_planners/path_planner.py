import scipy
import numpy as np
class PathPlanner:

    def generate_path(self):
        NotImplementedError

    def _step(self):
        NotImplementedError

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

    # def check_convergence(self, target_pos, position):
    #     #NOTE: do we need this check? should it be in the generate_path func?
    #     # check if we are within 1cm of our target by the end of our path
    #     dist = np.sqrt(np.sum((target_pos - position[-1])**2))
    #     if dist > 0.001:
    #         self.n_timesteps += 10
    #         return False
    #     else:
    #         return True
