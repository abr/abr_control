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
    print('\npydmps library required, see ' +
          'https://github.com/studywolf/pydmps\n')

import scipy.interpolate


class dmpFilter():
    def __init__(self):
        # create a dmp for a straight reach with a bell shaped velocity profile
        x = np.linspace(0, np.pi*2, 100)
        a = 1
        b = np.pi
        c = 1
        g = a * np.exp(-(x-b)**2/(2*c)**2)
        g /= np.sum(g)
        y_des = np.cumsum(g)
        self.dmps = pydmps.DMPs_discrete(n_dmps=3, n_bfs=50, dt=.01)
        y_des = np.vstack([y_des, y_des, y_des])
        self.dmps.imitate_path(y_des)

    @property
    def params(self):
        params = {'source': 'dmpFilter'}
        return params

    def plot_trajectory(self):
        self.dmps.reset_state()
        y,_,_ = self.dmps.rollout()
        self.dmps.reset_state()

        import matplotlib.pyplot as plt
        plt.plot(y, 'x')
        plt.show()

    def generate_path_function(self, target_xyz, start_xyz, time_limit,
            target_vel=False, rollout=None):
        self.start_xyz = start_xyz
        self.dmps.reset_state()
        self.dmps.goal = target_xyz - self.start_xyz
        trajectory,vel,_ = self.dmps.rollout(rollout)
        trajectory = np.array([traj + self.start_xyz for traj in trajectory])
        times = np.linspace(0, time_limit, len(trajectory))
        x = scipy.interpolate.interp1d(times, trajectory[:,0])
        y = scipy.interpolate.interp1d(times, trajectory[:,1])
        z = scipy.interpolate.interp1d(times, trajectory[:,2])
        if target_vel:
            dx = scipy.interpolate.interp1d(times, vel[:,0])
            dy = scipy.interpolate.interp1d(times, vel[:,1])
            dz = scipy.interpolate.interp1d(times, vel[:,2])
            self.path_func = [x,y,z,dx,dy,dz]
        else:
            self.path_func = [x,y,z]

    def next_timestep(self,t):
        target = []
        for dim in self.path_func:
            target.append(dim(t))
        return target

    def reset(self, target_xyz, start_xyz):
        self.start_xyz = start_xyz
        self.dmps.reset_state()
        self.dmps.goal = target_xyz - self.start_xyz

    def step(self, error):
        # get the next point in the target trajectory from the dmp
        target_xyz = np.copy(self.dmps.step(error=error*0.65e1)[0])
        target_xyz += self.start_xyz
        return target_xyz


if __name__ == '__main__':

    path = dmpFilter()

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    def gauss(a, b, c):
        return a * np.exp(-(x-b)**2/(2*c)**2)

    x = np.linspace(0, np.pi*6, 100)
    g = gauss(1, np.pi, 1)
    h = gauss(2, 3*np.pi, 1)
    i = gauss(1, 5*np.pi, 1)

    y_des = np.vstack([
        np.copy(x),
        np.cumsum(g - h + i),
        np.copy(x)])

    path.dmps.imitate_path(y_des)

    fig = plt.figure()
    fig.add_subplot(111, projection='3d')

    targets = np.array([
        [1, 1, 1],
        [3, 5, 3],
        [-2, -3, 2],
        ])

    for target in targets:
        path.generate_path_function(
            target,
            np.array([0, 0, 0]),
            1)
        y = np.zeros((100, 3))
        for ii, t in enumerate(np.linspace(0, 1, 100)):
            y[ii] = path.next_timestep(t)
        plt.plot(y[:, 0], y[:, 1], y[:, 2])

    plt.show()
