import numpy as np
"""
A function defining the shape of our path from start to target
Restrictions
------------
- the path must start at [0, 0, 0] and end at [1, 1, 1]
    these are the start and target position, respectively
    The reason for this is that we just need to define the shape of
    our path with respect to the straight line path from start to
    target. The path planner will do the stretching so that the path
    at t==0 will be the start, and will end at the target
"""
class Linear():
    def __init__(self, tol=1e-6):
        pass

    def step(self, t):
        return np.array([t, t, t])


class SinCurve():
    def __init__(self, axes=None, cycles=None):
        if axes is None:
            axes = ['x']
        if cycles is None:
            cycles = [1, 1, 1]
        self.axes = axes
        self.cycles = cycles

    def step(self, t):
        if 'x' in self.axes:
            x = np.sin(self.cycles[0] * t*np.pi/2)
        else:
            x =t

        if 'y' in self.axes:
            y = np.sin(self.cycles[1] * t*np.pi/2)
        else:
            y =t

        if 'z' in self.axes:
            z = np.sin(self.cycles[2] * t*np.pi/2)
        else:
            z =t

        return np.array([x, y, z])


class FromPoints():
    """
    Generates a position profile function from a set of points
    """
    def __init__(self, pts, n_sample_points=1000):
        # interpolate into function
        if pts.shape[0] != 3:
            pts = pts.T
        x = np.linspace(0, 1, n_sample_points)

        self.X = scipy.interpolate.interp1d(x, pts[0])
        self.Y = scipy.interpolate.interp1d(x, pts[1])
        self.Z = scipy.interpolate.interp1d(x, pts[2])

    def step(self, t):
        if t == 0:
            return np.zeros(3)
        if t == 1:
            return np.ones(3)

        xyz = np.array([self.X(t), self.Y(t), self.Z(t)])
        return xyz
