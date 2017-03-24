import numpy as np

from . import interface
from . import pygame_display
from ..arms.threelink import py3LinkArm


class interface(interface.interface):
    """ An interface for MapleSim models that have been exported to
    C and turned into shared libraries using Cython. Plots the movement
    in PyGame.
    """

    def __init__(self, robot_config, dt=.001, q_init=None, **kwargs):

        self.kwargs = kwargs

        super(interface, self).__init__(robot_config)

        self.q = np.zeros(self.robot_config.num_joints)  # joint angles
        self.dq = np.zeros(self.robot_config.num_joints)  # joint_velocities

        if q_init is not None:
            self.q_init = np.zeros(self.robot_config.num_joints*2)
            self.q_init[::2] = q_init
            # TODO: add in ability to set starting velocity, if useful
            # self.q_init[1::2] = dq_init
        else:
            self.q_init = None

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times forces have been sent

    def connect(self):
        """ Creates the MapleSim model and set up PyGame.
        """

        # create the PyGame display
        # TODO: read the arm lengths out of the config
        self.display = pygame_display.display(L=[2, 1.2, .7],
                                              **self.kwargs)

        # stores information returned from maplesim
        self.state = np.zeros(7)
        # maplesim arm simulation
        self.sim = py3LinkArm.pySim(dt=1e-5)

        self.sim.reset(self.state, self.q_init)
        self._update_state()
        print('Connected to MapleSim model')

    def disconnect(self):
        """ Reset the simulation and close PyGame display.
        """

        state = np.hstack([
            self.robot_config.rest_angles,
            np.zeros(self.robot_config.num_joints)])

        self.sim.reset(self.state, state)
        self._update_state()

        self.display.close()

        print('connection closed...')

    def send_forces(self, u, dt=None):
        """ Apply the specified forces to the robot,
        move the simulation one time step forward, and update
        the plot.

        u np.array: an array of the torques to apply to the robot
        """

        dt = self.dt if dt is None else dt
        u = -1 * np.array(u, dtype='float')

        for ii in range(int(np.ceil(dt/1e-5))):
            self.sim.step(self.state, u)
        self._update_state()

    def get_feedback(self):
        """ Return a dictionary of information needed by the controller. """

        return {'q': self.q,
                'dq': self.dq}

    def get_xyz(self, name):
        raise NotImplementedError("Not an available method" +
                                  "in the MapleSim interface")

    def set_target(self, xyz):
        """ Set the position of the target object.

        xyz np.array: the [x,y,z] location of the target (in meters)
        """
        self.display.set_target(xyz[:2])

    def _position(self):
        """Compute x,y position of the hand
        """

        xy = [self.robot_config.Tx('joint%i' % ii, q=self.q)
              for ii in range(self.robot_config.num_joints)]
        xy = np.vstack([xy, self.robot_config.Tx('EE', q=self.q)])
        self.joints_x = xy[:, 0]
        self.joints_y = xy[:, 1]
        return np.array([self.joints_x, self.joints_y])

    def _update_state(self):
        """Update the local variables"""
        self.t = self.state[0]
        self.q = self.state[1:4]
        self.dq = self.state[4:]
        self._position()
        self.x = np.array([self.joints_x[-1], self.joints_y[-1]])

        # update the display
        self.display.update(self.q)
