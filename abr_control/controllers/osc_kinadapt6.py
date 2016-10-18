import numpy as np

from . import osc
from . import osc_dynadapt
from .keeplearningsolver import KeepLearningSolver


class controller(osc_dynadapt.controller):
    """ Extension of the OSC controller that incorporates dynamics
    adaptation using a Nengo model
    """

    def __init__(self, robot_config, **kwargs):

        super(controller, self).__init__(robot_config, **kwargs)

        self.kd = self.lamb = 10

        self.radius = np.sqrt(self.robot_config.num_joints * 2) * 10)
        self.lower_bound = self.radius - .1 * self.radius
        self.upper_bound = self.radius + .1 * self.radius

    def control(self, q, dq, target_state, object_xyz):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_state np.array: the current target for the end-effector
        """

        self.q = q
        self.dq = dq

        x_d = target_state[:3]  # desired position
        dx_d = target_state[3:6]  # desired velocity
        ddx_d = target_state[6:]  # desired acceleration

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J('objectEE', q)
        dx = np.dot(JEE, dq)

        s = (dx - dx_d) + self.lamb * (object_xyz - x_d)

        ar = self.lamb * (dx - dx_d) - ddx_d

        def m_func(x):
            if x < self.lower_bound:
                return 0
            elif x > self.upper_bound:
                return 1
            return ((x - self.lower_bound) /
                    (self.upper_bound - self.lower_bound))
        norm = (np.linalg.norm(np.hstack([q, dq])))
        print('norm: ', norm)
        m = m_func(norm)

        def sat(x):

        s_delta = s - self.lower_bound * 

        self.training_signal = -self.ka * (1 - m) * s_delta
        self.sim.run(time_in_seconds=.001, progress_bar=False)
        f_hat = self.u_adapt

        u = -self.kd * s - ar + (1 - m) * f_hat + m * u_sl

        return u
