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

try:
    import pydmps
except ImportError:
    print('\npydmps library required, see github.com/studywolf/pydmps\n')

import scipy.interpolate


class BellShaped():
    def __init__(self, error_scale=1):
        self.error_scale = error_scale
        # create a dmp for a straight reach with a bell shaped velocity profile
        x = np.linspace(0, np.pi*2, 100)
        a = 1  # amplitude
        b = np.pi  # center
        c = 1  # std deviation
        g = a * np.exp(-(x-b)**2/(2*c)**2)
        g /= np.sum(g)  # normalize
        # integrate desired velocities to get desired positions over time
        y_des = np.cumsum(g)
        # want to follow the same trajectory in (x, y, z)
        y_des = np.vstack([y_des, y_des, y_des])

        self.dmps = pydmps.DMPs_discrete(n_dmps=3, n_bfs=50, dt=.01)
        self.dmps.imitate_path(y_des)


    def plot_trajectory(self):
        """
        Once the dmp function has been generated, calling this function will
        plot the profile of the path planner
        """
        self.dmps.reset_state()
        y, _, _ = self.dmps.rollout()
        self.dmps.reset_state()

        import matplotlib.pyplot as plt
        plt.plot(y, 'x')
        plt.show()


    def generate_path_function(self, target_pos, pos, time_limit,
                               timesteps=None):
        """
        Generates a path function from state to target_pos and interpolates it
        with respect to the time_limit. The function can then be stepped through
        to reach a target_pos within the specified time.

        PARAMETERS
        ----------
        target_pos: list of 3 floats
            the target end-effector position in cartesian coordinates [meters]
        pos : numpy.array
            the current position of the system
        time_limit: float
            the desired time to go from state to target_pos [seconds]
        timesteps: int, Optional (Default: None)
            how many steps to run the dmp, which returns position and velocity
            information. Increasing the length will extend the tail of the path
            that is planned.
        """
        self.reset(target_pos=target_pos, pos=pos)

        trajectory, vel, _ = self.dmps.rollout(timesteps=timesteps)
        trajectory = np.array([traj + self.pos for traj in trajectory])

        times = np.linspace(0, time_limit, len(trajectory))
        pos_path = []
        vel_path = []
        for dim in range(3):
            pos_path.append(scipy.interpolate.interp1d(
                times, trajectory[:, dim]))
            vel_path.append(scipy.interpolate.interp1d(\
                times, vel[:, dim]))

        self.path_func = np.hstack((pos_path, vel_path))

    def next_timestep(self, t):
        """
        Called after `generate_path_function()` has been run to create the path
        planner function. The interpolated function is called and the path at
        time t is returned

        PARAMETERS
        ----------
        t: float
            time in seconds along the planned path
            ex: if generate_path_function is called with a 10 second limit,
            passing 5 in to next_timestep will return the position of the EE
            half-way (wrt time) through the generated path
        """
        target = []
        target_vel = []
        for ii in range(3):
            target.append(self.path_func[ii](t))
        for ii in range(3, 6):
            target_vel.append(self.path_func[ii](t))
        return target, target_vel


    def reset(self, target_pos, pos):
        """
        Resets the dmp path planner to a new state and target_pos

        PARAMETERS
        ----------
        target_pos: list of 3 floats
            the target_pos end-effector position in cartesian coordinates [meters]
        pos: list of 3 floats
            the current end-effector cartesian position [meters]
        """
        self.pos = pos
        self.dmps.reset_state()
        self.dmps.goal = target_pos - self.pos


    def step(self, error=None):
        """
        Steps through the dmp, returning the next position and velocity along
        the path planner.
        """
        if error is None:
            error = 0
        # get the next point in the target trajectory from the dmp
        target, target_vel, _ = self.dmps.step(error=error * self.error_scale)
        # add the start position offset since the dmp starts from the origin
        target = target + self.pos
        return target, target_vel
