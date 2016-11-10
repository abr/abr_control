import numpy as np

from . import interface
from . import pygame_display
from ..arms.threelink import py3LinkArm


class interface(interface.interface):
    """ An interface for MapleSim models that have been exported to
    C and turned into shared libraries using Cython. Plots the movement
    in PyGame.
    """

    def __init__(self, robot_config, dt=.001):
        super(interface, self).__init__(robot_config)

        self.q = np.zeros(self.robot_config.num_joints)  # joint angles
        self.dq = np.zeros(self.robot_config.num_joints)  # joint_velocities

        self.dt = dt  # time step
        self.count = 0  # keep track of how many times apply_u has been called

    def connect(self):
        """ Creates the MapleSim model and set up PyGame.
        """

        # stores information returned from maplesim
        self.state = np.zeros(7)
        # maplesim arm simulation
        self.sim = py3LinkArm.pySim(dt=1e-5)

        self.sim.reset(self.state)
        self._update_state()
        print('Connected to MapleSim model')

        # create the PyGame display
        self.display = pygame_display.display(L=self.robot_config.L)
        # update to starting position
        self.display.update(q=self.q)

    def disconnect(self):
        """ Reset the simulation and close PyGame display.
        """

        state = np.zeros(self.DOF*2)
        state[::2] = self.init_q if not q else np.copy(q)
        state[1::2] = self.init_dq if not dq else np.copy(dq)

        self.sim.reset(self.state, state)
        self._update_state()

        self.display.close()

        print('connection closed...')

    def apply_u(self, u):
        """ Apply the specified torque to the robot joints,
        move the simulation one time step forward, and update
        the plot.

        u np.array: an array of the torques to apply to the robot
        """

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

    def _position(self):
        """Compute x,y position of the hand
        """
        q0 = self.q[0]
        q1 = self.q[1]
        q2 = self.q[2]

        self.joints_x = np.cumsum([
            0,
            self.L[0] * np.cos(q0),
            self.L[1] * np.cos(q0+q1),
            self.L[2] * np.cos(q0+q1+q2)])
        self.joints_y = np.cumsum([
            0,
            self.L[0] * np.sin(q0),
            self.L[1] * np.sin(q0+q1),
            self.L[2] * np.sin(q0+q1+q2)])
        return np.array([self.joints_x, self.joints_y])

    def _update_state(self):
        """Update the local variables"""
        self.t = self.state[0]
        self.q = self.state[1:4]
        self.dq = self.state[4:]
        self._position()
        self.x = np.array([self.joints_x[-1], self.joints_y[-1]])
