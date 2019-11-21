"""
Given two points, a path is drawn between them along the perimiter of a circle.
If the two points are of different radius from the origin, then two circles are
generated and they're paths are linearly combined to join the points. The shortest
path around the circle is taken. A linear path along z is taken.
"""

import numpy as np
import matplotlib.pyplot as plt

try:
    import pydmps
except ImportError:
    print('\npydmps library required, see github.com/studywolf/pydmps\n')

from .first_order_arc import FirstOrderArc


class FirstOrderArcDMP(FirstOrderArc):
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
        dt = 1/self.n_timesteps

        super(FirstOrderArcDMP, self).__init__(n_timesteps=self.n_timesteps)
        pos, vel = super(FirstOrderArcDMP, self).generate_path(
            position=np.ones(3), target_pos=np.ones(3))

        self.pos_dmp = pydmps.DMPs_discrete(n_dmps=3, n_bfs=50, dt=dt)
        self.pos_dmp.imitate_path(pos.T)


    def generate_path(self, position, target_pos, plot=False):
        """
        Calls the step function self.n_timestep times to pregenerate
        the entire path planner

        PARAMETERS
        ----------
        position: numpy.array
            the current position of the system
        target_pos: numpy.array
            the target position
        plot: boolean, optional (Default: False)
            plot the path after generating if True
        """
        self.reset(target_pos=target_pos, position=position)

        self.position, self.velocity, _ = self.pos_dmp.rollout(
            timesteps=self.n_timesteps)
        self.position = np.array([traj + self.origin for traj in self.position])

        # reset trajectory index
        self.n = 0

        if plot:
            plt.plot(self.position)
            plt.legend(['X', 'Y', 'Z'])
            plt.show()

        return self.position, self.velocity


    def reset(self, target_pos, position):
        """
        Resets the dmp path planner to a new state and target_pos

        PARAMETERS
        ----------
        target_pos: list of 3 floats
            the target_pos end-effector position in cartesian coordinates [meters]
        position: list of 3 floats
            the current end-effector cartesian position [meters]
        """
        self.origin = position
        self.pos_dmp.reset_state()
        self.pos_dmp.goal = target_pos - self.origin
        # self.vel_dmp.reset_state()
        # self.vel_dmp.goal = np.zeros(3)


    def _step(self, error=None):
        """
        Steps through the dmp, returning the next position and velocity along
        the path planner.
        """
        if error is None:
            error = 0
        # get the next point in the target trajectory from the dmp
        position, velocity, _ = self.pos_dmp.step(error=error * self.error_scale)
        # add the start position offset since the dmp starts from the origin
        position = position + self.origin

        return position, velocity
