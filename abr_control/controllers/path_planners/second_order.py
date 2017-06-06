import numpy as np

from .path_planner import PathPlanner


class SecondOrder(PathPlanner):
    """ Implement a trajectory controller on top of a controller

    Implements an trajectory controller on top of a given
    point to point control system. Returns a set of desired positions
    and velocities.

    Parameters
    ----------
    robot_config : class instance
        passes in all relevant information about the arm
        from its config, such as: number of joints, number
        of links, mass information etc.
    """

    def __init__(self, robot_config):
        super(SecondOrder, self).__init__(robot_config)

    def generate(self, state, target, dy=np.zeros(3), fc1=200, fc2=0.005,
                 n_timesteps=100, dt=0.005):
        """ filter the target so that it doesn't jump, but moves smoothly

        Parameters
        ----------
        dy : numpy.array, optional (Default: numpy.zeros(3))
            cartesian velocity [meters]
        state : numpy.array
            the current position of the system [meters]
        target : numpy.array
            the target position [radians]
        fc1 : float, optional (Default:200)
            filter constant 1
        fc2 : float, optional (Default:0.005)
            filter constant 2
        n_timesteps : int, optional (Default: 100)
            the number of time steps to reach the target
        dt : float, optional (Default: 0.005)
            the time step for the filter [seconds]

        Attributes
        ----------
        y : numpy.array
            position cartesian coordinates [meters]
        """

        self.y = []
        self.dy = []

        # initialize the filters to the starting position of the end effector
        y = state
        dy = state

        for ii in range(0, n_timesteps):
            # if filtered target is within 2cm of the target, use target
            #TODO have to account for dt somewhere, used loop time previously
            # to implicitly add it
            if np.linalg.norm(y - target) > 0.002:
                dy += (-1.0 / fc1 * dy - fc2 * y + (2. / fc1) * target) * dt
                print('dy: ', dy)
                y += (-1.0 / fc1 * y + fc2 * dy) * dt
                print('y: ', y)
            else:
                y = target
            self.y.append(y)
            print('next: ', self.y[ii])
            self.dy.append(dy)
        # reset trajectory index
        self.n = 0
        self.n_timesteps = n_timesteps

        import matplotlib
        matplotlib.use("TKAgg")
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(range(0,len(self.y)), np.array(self.y)[:,0], 'r', label='x')
        plt.plot(range(0,len(self.y)), np.array(self.y)[:,1], 'b', label='y')
        plt.plot(range(0,len(self.y)), np.array(self.y)[:,2], 'k', label='z')
        plt.plot(np.arange(len(self.y)), np.ones(len(self.y)) * target[0],
                 '--r', label = 'target_x')
        plt.plot(np.arange(len(self.y)), np.ones(len(self.y)) * target[1],
                 '--b', label = 'target_y')
        plt.plot(np.arange(len(self.y)), np.ones(len(self.y)) * target[2],
                 '--k', label = 'target_z')
        plt.legend()
        plt.show()

    def next(self):
        """ Return the next target point along the generated trajectory """

        # get the next target state if we're not at the end of the trajectory
        self.target = (self.y[self.n] #  np.hstack([self.y[self.n], self.dy[self.n]])
                       if self.n < self.n_timesteps else self.target)
        self.n += 1

        return self.target
