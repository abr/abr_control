import cloudpickle
import numpy as np
import os
import sympy as sp

import abr_control.arms


class robot_config():
    """ Class defines a bunch of useful functions for controlling
    a given robot, including transformation to joints and COMs,
    Jacobians, the inertia matrix in joint space, and the effects
    of gravity. Uses SymPy and lambdify to do this.
    """

    def __init__(self, num_joints, num_links, robot_name="robot",
                 regenerate_functions=False):
        """
        num_joints int: number of joints in robot
        num_links int: number of arm segments in robot
        robot_name string: used for saving/loading functions to file
        regenerate_functions boolean: if True, don't look to saved files
                                      regenerate all transforms and Jacobians
        """

        self.num_joints = num_joints
        self.num_links = num_links
        self.robot_name = robot_name
        self.config_folder = (os.path.dirname(abr_control.arms.__file__) +
                              '/%s/saved_functions' % robot_name)

        self.regenerate_functions = regenerate_functions

        # create function dictionaries
        self._Tx = {}  # for transform calculations
        self._T_inv = {}  # for inverse transform calculations
        self._J = {}  # for Jacobian calculations
        self._M = []  # placeholder for (x,y,z) inertia matrices
        self._Mq = None  # placeholder for joint space inertia matrix function
        self._Mq_g = None  # placeholder for joint space gravity term function

        # set up our joint angle symbols
        self.q = [sp.Symbol('q%i' % ii) for ii in range(self.num_joints)]
        self.dq = [sp.Symbol('dq%i' % ii) for ii in range(self.num_joints)]
        # set up an (x,y,z) offset
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]

        self.gravity = sp.Matrix([[0, 0, -9.81, 0, 0, 0]]).T

    def J(self, name, q, x=[0, 0, 0]):
        """ Calculates the Jacobian for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Jacobian function
        x list: the [x,y,z] position of interest in "name"'s reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._J.get(funcname, None) is None:
            print('Generating Jacobian function for %s' % name)
            self._J[funcname] = self._calc_J(
                name=name, x=x, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(x)
        return np.array(self._J[funcname](*parameters), dtype='float32')

    def dJ(self, name, q, x=[0, 0, 0]):
        """ Calculates the derivative of a Jacobian for a joint or link
        with respect to time

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Jacobian function
        x list: the [x,y,z] position of interest in "name"'s reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._dJ.get(funcname, None) is None:
            print('Generating derivative of Jacobian ',
                  'function for %s' % funcname)
            self._dJ[funcname] = self._calc_dJ(
                name=name, x=x, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(x)
        return np.array(self._dJ[funcname](*parameters), dtype='float32')

    def Mq(self, q):
        """ Calculates the joint space inertia matrix for the ur5

        q list: set of joint angles to pass in to the Mq function
        """
        # check for function in dictionary
        if self._Mq is None:
            print('Generating inertia matrix function')
            self._Mq = self._calc_Mq(regenerate=self.regenerate_functions)
        parameters = tuple(q)
        return np.array(self._Mq(*parameters), dtype='float32')

    def Mq_g(self, q):
        """ Calculates the force of gravity in joint space for the ur5

        q list: set of joint angles to pass in to the Mq_g function
        """
        # check for function in dictionary
        if self._Mq_g is None:
            print('Generating gravity effects function')
            self._Mq_g = self._calc_Mq_g(regenerate=self.regenerate_functions)
        parameters = tuple(q)
        return np.array(self._Mq_g(*parameters), dtype='float32').flatten()

    def Tx(self, name, q, x=[0, 0, 0]):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the T function
        x list: the [x,y,z] position of interest in "name"'s reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._Tx.get(funcname, None) is None:
            print('Generating transform function for %s' % name)
            # TODO: link0 and joint0 share a transform, but will
            # both have their own transform calculated with this check
            self._Tx[funcname] = self._calc_Tx(
                name, x=x, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(x)
        return self._Tx[funcname](*parameters)[:-1].flatten()

    def T_inv(self, name, q, x=[0, 0, 0]):
        """ Calculates the inverse transform for a joint or link

        q list: set of joint angles to pass in to the T function
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._T_inv.get(funcname, None) is None:
            print('Generating inverse transform function for % s' % funcname)
            self._T_inv[funcname] = self._calc_T_inv(
                name=name, x=x, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(x)
        return self._T_inv[funcname](*parameters)

    def _calc_dJ(self, name, x, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the derivative of the Jacobian
        for a joint or link with respect to time

        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our Jacobian saved in file
        if (regenerate is False and
                os.path.isfile('%s/%s.dJ' % (self.config_folder, filename))):
            dJ = cloudpickle.load(open('%s/%s.dJ' %
                                       (self.config_folder, filename), 'rb'))
        else:
            J = self._calc_J(name, x=x, lambdify=False)
            dJ = sp.Matrix(np.zeros(J.shape, dtype='float32'))
            # calculate derivative of (x,y,z) wrt to time
            # which each joint is dependent on
            for ii in range(J.shape[0]):
                for jj in range(J.shape[1]):
                    for kk in range(self.num_joints):
                        dJ[ii, jj] += sp.simplify(
                            J[ii, jj].diff(self.q[kk]))
            dJ = sp.simplify(dJ)

            # save to file
            cloudpickle.dump(dJ, open('%s/%s.dJ' %
                                      (self.config_folder, filename), 'wb'))

        dJ = sp.Matrix(dJ).T  # correct the orientation of J
        if lambdify is False:
            return dJ
        return sp.lambdify(self.q + self.x, dJ, "numpy")

    def _calc_J(self, name, x, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the Jacobian for a joint or link

        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our Jacobian saved in file
        if (regenerate is False and
                os.path.isfile('%s/%s.J' % (self.config_folder, filename))):
            J = cloudpickle.load(open('%s/%s.J' %
                                      (self.config_folder, filename), 'rb'))
        else:
            Tx = self._calc_Tx(name, x=x, lambdify=False)
            J = []
            # calculate derivative of (x,y,z) wrt to each joint
            for ii in range(self.num_joints):
                J.append([])
                J[ii].append(sp.simplify(Tx[0].diff(self.q[ii])))  # dx/dq[ii]
                J[ii].append(sp.simplify(Tx[1].diff(self.q[ii])))  # dy/dq[ii]
                J[ii].append(sp.simplify(Tx[2].diff(self.q[ii])))  # dz/dq[ii]

            end_point = name.strip('link').strip('joint')
            if 'EE' not in end_point:
                end_point = min(int(end_point) + 1, self.num_joints)
                # add on the orientation information up to the last joint
                for ii in range(end_point):
                    J[ii] = J[ii] + self.J_orientation[ii]
                # fill in the rest of the joints orientation info with 0
                for ii in range(end_point, self.num_joints):
                    J[ii] = J[ii] + [0, 0, 0]
            J = sp.simplify(J)

            # save to file
            cloudpickle.dump(J, open('%s/%s.J' %
                                     (self.config_folder, filename), 'wb'))

        J = sp.Matrix(J).T  # correct the orientation of J
        if lambdify is False:
            return J
        return sp.lambdify(self.q + self.x, J, "numpy")

    def _calc_Mq(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the inertia matrix in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        # check to see if we have our inertia matrix saved in file
        if (regenerate is False and
                os.path.isfile('%s/Mq' % self.config_folder)):
            Mq = cloudpickle.load(open('%s/Mq' % self.config_folder, 'rb'))
        else:
            # get the Jacobians for each link's COM
            J = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]

            # transform each inertia matrix into joint space
            # sum together the effects of arm segments' inertia on each motor
            Mq = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                Mq += (J[ii].T * self._M[ii] * J[ii])
            Mq = sp.simplify(Mq)

            # save to file
            cloudpickle.dump(Mq, open('%s/Mq' % self.config_folder, 'wb'))

        if lambdify is False:
            return Mq
        return sp.lambdify(self.q, Mq, "numpy")

    def _calc_Mq_g(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the force of gravity in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        # check to see if we have our gravity term saved in file
        if (regenerate is False and
                os.path.isfile('%s/Mq_g' % self.config_folder)):
            Mq_g = cloudpickle.load(open('%s/Mq_g' %
                                         self.config_folder, 'rb'))
        else:
            # get the Jacobians for each link's COM
            J = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]

            # transform each inertia matrix into joint space and
            # sum together the effects of arm segments' inertia on each motor
            Mq_g = sp.zeros(self.num_joints, 1)
            for ii in range(self.num_joints):
                Mq_g += sp.simplify(J[ii].T * self._M[ii] * self.gravity)
            Mq_g = sp.simplify(Mq_g)

            # save to file
            cloudpickle.dump(Mq_g, open('%s/Mq_g' % self.config_folder, 'wb'))

        if lambdify is False:
            return Mq_g
        return sp.lambdify(self.q, Mq_g, "numpy")

    def _calc_T(self, name):
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """
        raise NotImplementedError("_calc_T function not implemented")

    def _calc_Tx(self, name, x=None, lambdify=True, regenerate=False):  # noqa C907
        """ Uses Sympy to transform x from the reference frame of a joint
        or link to the origin (world) coordinates.

        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our transformation saved in file
        if (regenerate is False and
                os.path.isfile('%s/%s.T' % (self.config_folder, filename))):
            Tx = cloudpickle.load(open('%s/%s.T' %
                                       (self.config_folder, filename), 'rb'))
        else:
            T = self._calc_T(name=name)
            # transform x into world coordinates
            if np.allclose(x, 0):
                # if we're only interested in the origin, not including
                # the x variables significantly speeds things up
                Tx = sp.simplify(T * sp.Matrix([0, 0, 0, 1]))
            else:
                # if we're interested in other points in the given frame
                # of reference, calculate transform with x variables
                Tx = sp.simplify(T * sp.Matrix(self.x + [1]))
                print('what are you doing here')

            # save to file
            cloudpickle.dump(Tx, open('%s/%s.T' %
                                      (self.config_folder, filename), 'wb'))

        if lambdify is False:
            return Tx
        return sp.lambdify(self.q + self.x, Tx, "numpy")

    def _calc_T_inv(self, name, x, lambdify=True, regenerate=False):
        """ Return the inverse transform matrix, which converts from
        world coordinates into the robot's end-effector reference frame

        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our transformation saved in file
        if (regenerate is False and
                os.path.isfile('%s/%s.T_inv' % (self.config_folder,
                                                filename))):
            T_inv = cloudpickle.load(open('%s/%s.T_inv' %
                                          (self.config_folder,
                                           filename), 'rb'))
        else:
            T = self._calc_T(name=name)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]]))

            # save to file
            cloudpickle.dump(T_inv, open('%s/%s.T_inv' %
                                         (self.config_folder,
                                          filename), 'wb'))

        if lambdify is False:
            return T_inv
        return sp.lambdify(self.q + self.x, T_inv, "numpy")
