"""
Running operational space control with a PyGame display, and using the pydmps
library to specify a trajectory for the end-effector to follow, in
this case, a circle.
To install the pydmps library, clone https://github.com/studywolf/pydmps
and run 'python setup.py develop'
"""
import numpy as np

try:
    import pydmps
except ImportError:
    print('\npydmps library required, see github.com/studywolf/pydmps\n')

import scipy.interpolate


class BellShaped():
    def __init__(self):
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


    @property
    def params(self):
        params = {'source': 'dmpFilter'}
        return params


    def plot_trajectory(self):
        """
        """
        self.dmps.reset_state()
        y, _, _ = self.dmps.rollout()
        self.dmps.reset_state()

        import matplotlib.pyplot as plt
        plt.plot(y, 'x')
        plt.show()


    def generate_path_function(self, target, state, time_limit, rollout=None):
        """

        """
        self.state = state
        self.dmps.reset_state()
        self.dmps.goal = target - self.state

        trajectory, vel, _ = self.dmps.rollout(rollout)
        trajectory = np.array([traj + self.state[:3] for traj in trajectory])

        times = np.linspace(0, time_limit, len(trajectory))
        x = scipy.interpolate.interp1d(times, trajectory[:, 0])
        y = scipy.interpolate.interp1d(times, trajectory[:, 1])
        z = scipy.interpolate.interp1d(times, trajectory[:, 2])
        dx = scipy.interpolate.interp1d(times, vel[:, 0])
        dy = scipy.interpolate.interp1d(times, vel[:, 1])
        dz = scipy.interpolate.interp1d(times, vel[:, 2])

        self.path_func = [x, y, z, dx, dy, dz]

    def next_timestep(self, t):
        """
        """
        target = np.zeros(len(self.path_func))
        for ii, interp_function in enumerate(self.path_func):
            target[ii] = interp_function(t)
        return target


    def reset(self, target, state):
        """
        """
        self.state = state
        self.dmps.reset_state()
        self.dmps.goal = target - self.state


    def step(self, error):
        """
        """
        # get the next point in the target trajectory from the dmp
        target = np.copy(self.dmps.step(error=error*0.65e1)[0])
        target += self.state
        return target
