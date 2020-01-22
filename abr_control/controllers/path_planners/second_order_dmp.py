"""
Running operational space control with a PyGame display, and using the pydmps
library to specify a trajectory for the end-effector to follow, in
this case, a bell shaped velocity profile.
To install the pydmps library, clone https://github.com/studywolf/pydmps
and run 'python setup.py develop'

***NOTE*** there are two ways to use this filter
1: wrt to timesteps
- the dmp is created during the instantiation of the class and the next step
along the path is returned by calling the `step()` function

2: wrt to time
- after instantiation, calling `generate_path_function()` interpolates the dmp
to the specified time limit. Calling the `next_timestep(t)` function at a
specified time will return the end-effector state at that point along the path
planner. This ensures that the path will reach the desired target within the
time_limit specified in `generate_path_function()`

"""
import numpy as np
import matplotlib.pyplot as plt

try:
    import pydmps
except ImportError:
    print("\npydmps library required, see github.com/studywolf/pydmps\n")

from .path_planner import PathPlanner


class SecondOrderDMP(PathPlanner):
    """
    PARAMETERS
    ----------
    n_timesteps: int, Optional (Default: 3000)
        the number of steps to break the path into
    error_scale: int, Optional (Default: 1)
        the scaling factor to apply to the error term, increasing error passed
        1 will increase the speed of motion
    """

    def __init__(self, n_timesteps=3000, error_scale=1):
        self.n_timesteps = n_timesteps
        self.error_scale = error_scale

        # create a dmp for a straight reach with a bell shaped velocity profile
        x = np.linspace(0, np.pi * 2, 100)
        a = 1  # amplitude
        b = np.pi  # center
        c = 1  # std deviation
        g = a * np.exp(-((x - b) ** 2) / (2 * c) ** 2)
        g /= np.sum(g)  # normalize
        # integrate desired velocities to get desired positions over time
        y_des = np.cumsum(g)
        # want to follow the same trajectory in (x, y, z)
        y_des = np.vstack([y_des, y_des, y_des])

        # we can control the DMP rollout speed with the time step size
        # the DMP will reach the target in 1s of sim time
        dt = 1 / n_timesteps
        self.dmps = pydmps.DMPs_discrete(n_dmps=3, n_bfs=50, dt=dt)
        self.dmps.imitate_path(y_des)

    def generate_path(self, position, target_position, plot=False):
        """
        Calls the step function self.n_timestep times to pregenerate
        the entire path planner

        PARAMETERS
        ----------
        position: numpy.array
            the current position of the system
        target_position: numpy.array
            the target position
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """
        self.reset(target_position=target_position, position=position)

        self.position_path, self.velocity_path, _ = self.dmps.rollout(
            timesteps=self.n_timesteps
        )
        self.position_path = np.array(
            [traj + self.origin for traj in self.position_path]
        )

        # reset trajectory index
        self.n = 0

        if plot:
            plt.plot(self.position_path)
            plt.legend(["X", "Y", "Z"])
            plt.show()

        return self.position_path, self.velocity_path

    def reset(self, target_position, position):
        """
        Resets the dmp path planner to a new state and target_position

        PARAMETERS
        ----------
        target_position: list of 3 floats
            the target_position end-effector position in cartesian coordinates [meters]
        position: list of 3 floats
            the current end-effector cartesian position [meters]
        """
        self.origin = position
        self.dmps.reset_state()
        self.dmps.goal = target_position - self.origin

    def _step(self, error=None):
        """
        Steps through the dmp, returning the next position and velocity along
        the path planner.
        """
        if error is None:
            error = 0
        # get the next point in the target trajectory from the dmp
        position, velocity, _ = self.dmps.step(error=error * self.error_scale)
        # add the start position offset since the dmp starts from the origin
        position = position + self.origin

        return position, velocity
