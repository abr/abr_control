import pickle
import numpy as np
import os
import sympy as sp
from sympy.utilities.autowrap import autowrap

import abr_control
import abr_control.arms


# TODO : store both the lambdified and unlambdified versions of everything

# TODO : check to see how long it takes to load T from file vs recompute

class robot_config():
    """ Class defines a bunch of useful functions for controlling
    a given robot, including transformation to joints and COMs,
    Jacobians, the inertia matrix in joint space, and the effects
    of gravity. Uses SymPy and lambdify to do this.
    """

    def __init__(self, num_joints, num_links, robot_name="robot",
                 regenerate_functions=False,
                 use_simplify=False, use_cython=False):
        """
        num_joints int: number of joints in robot
        num_links int: number of arm segments in robot
        robot_name string: used for saving/loading functions to file
        regenerate_functions boolean: if True, don't look to saved files
                                      regenerate all transforms and Jacobians
        use_simplify boolean: if True, symbolic representations are simplified
                              useful when execution time is more important 
                              than generation time
        use_cython boolean: if True, a more efficient function is generated
                            useful when execution time is more important than
                            generation time
        """

        self.num_joints = num_joints
        self.num_links = num_links
        self.robot_name = robot_name
        self.config_folder = (os.path.dirname(abr_control.arms.__file__) +
                              '/%s/saved_functions' % robot_name)

        self.regenerate_functions = regenerate_functions
        self.use_cython = use_cython
        self.simplify = sp.simplify if use_simplify is True else lambda x: x

        # create function dictionaries
        self._Tx = {}  # for transform calculations
        self._T_inv = {}  # for inverse transform calculations
        self._J = {}  # for Jacobian calculations
        self._M = []  # placeholder for (x,y,z) inertia matrices
        self._Mq = None  # placeholder for joint space inertia matrix function
        self._Mq_g = None  # placeholder for joint space gravity term function
        self._orientation = {} # placeholder for orientation functions
        self._T_inv = {}  # for inverse transform calculations
        self._Tx = {}  # for point transform calculations
        self._T_func = {}  # for transform matrix calculations

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

    def orientation(self, name, q):
        """ Uses Sympy to generate the orientation for a joint or link
        calculated as a quaternion.

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the Mq_g function
        regenerate boolean: if True, don't use saved functions
        """
        # get transform matrix for reference frame of interest
        if self._T_func.get(name, None) is None:
            # check to see if we have our transformation saved in file
            if (self.regenerate_functions is False and
                    os.path.isfile('%s/%s.T' % (self.config_folder, name))):
                T = pickle.load(open('%s/%s.T' %
                                          (self.config_folder, name),
                                          'rb'))
            else:
                T = self._calc_T(name=name)

                # save to file
                pickle.dump(sp.Matrix(T), open(
                    '%s/%s.T' % (self.config_folder, name), 'wb'))
            if self.use_cython is True:
                T_func = autowrap(T, backend="cython", args=self.q)
            else:
                T_func = sp.lambdify(self.q, T, "numpy")
            self._T_func[name] = T_func
        T = self._T_func[name](*q)

        return abr_control.utils.transformations.quaternion_from_matrix(T)

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
            dJ = pickle.load(open('%s/%s.dJ' %
                                       (self.config_folder, filename), 'rb'))
        else:
            J = self._calc_J(name, x=x, lambdify=False)
            dJ = sp.Matrix(np.zeros(J.shape, dtype='float32'))
            # calculate derivative of (x,y,z) wrt to time
            # which each joint is dependent on
            for ii in range(J.shape[0]):
                for jj in range(J.shape[1]):
                    for kk in range(self.num_joints):
                        dJ[ii, jj] += self.simplify(
                            J[ii, jj].diff(self.q[kk]))
            dJ = sp.Matrix(dJ).T

            # save to file
            pickle.dump(dJ, open(
                '%s/%s.dJ' % (self.config_folder, filename), 'wb'))

        if lambdify is False:
            return dJ
        if self.use_cython is True:
            return autowrap(dJ, backend="cython", args=self.q+self.x)
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
            J = pickle.load(open('%s/%s.J' %
                                      (self.config_folder, filename), 'rb'))
        else:
            Tx = self._calc_Tx(name, x=x, lambdify=False)
            # NOTE: calculating the Jacobian this way doesn't incur any
            # real computational cost (maybe 30ms) and it simplifies adding
            # the orientation information below (as opposed to using
            # sympy's Tx.jacobian method)
            J = []
            # calculate derivative of (x,y,z) wrt to each joint
            for ii in range(self.num_joints):
                J.append([])
                J[ii].append(self.simplify(Tx[0].diff(self.q[ii])))  # dx/dq[ii]
                J[ii].append(self.simplify(Tx[1].diff(self.q[ii])))  # dy/dq[ii]
                J[ii].append(self.simplify(Tx[2].diff(self.q[ii])))  # dz/dq[ii]

            end_point = name.strip('link').strip('joint')
            end_point = self.num_joints if 'EE' in end_point else end_point

            end_point = min(int(end_point) + 1, self.num_joints)
            # add on the orientation information up to the last joint
            for ii in range(end_point):
                J[ii] = J[ii] + list(self.J_orientation[ii])
            # fill in the rest of the joints orientation info with 0
            for ii in range(end_point, self.num_joints):
                J[ii] = J[ii] + [0, 0, 0]

            # save to file
            pickle.dump(sp.Matrix(J), open(
                '%s/%s.J' % (self.config_folder, filename), 'wb'))

        J = sp.Matrix(J).T  # correct the orientation of J
        if lambdify is False:
            return J
        if self.use_cython is True:
            return autowrap(J, backend="cython", args=self.q+self.x)
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
            Mq = pickle.load(open('%s/Mq' % self.config_folder, 'rb'))
        else:
            # get the Jacobians for each link's COM
            # TODO: make sure that we're not regenerating all these
            # Jacobians again here if they've already been regenerated
            # once, but that they are actually regenerated if only this is called
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_joints)]

            # sum together the effects of each arm segment's inertia
            print(self.num_joints)
            Mq = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                # transform each inertia matrix into joint space
                Mq += (J_links[ii].T * self._M_links[ii] * J_links[ii])
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.num_joints):
                # transform each inertia matrix into joint space
                Mq += (J_joints[ii].T * self._M_joints[ii] * J_joints[ii])
            Mq = self.simplify(Mq)
            Mq = sp.Matrix(Mq)

            # save to file
            pickle.dump(Mq, open(
                '%s/Mq' % self.config_folder, 'wb'))

        if lambdify is False:
            return Mq
        if self.use_cython is True:
            return autowrap(Mq, backend='cython', args=self.q)
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
            Mq_g = pickle.load(open('%s/Mq_g' %
                                         self.config_folder, 'rb'))
        else:
            # get the Jacobians for each link's COM
            # TODO: make sure that we're not regenerating all these
            # Jacobians again here if they've already been regenerated
            # once, but that they are actually regenerated if only this is called
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_joints)]

            # sum together the effects of each arm segment's inertia
            print(self.num_joints)
            Mq_g = sp.zeros(self.num_joints, 1)
            for ii in range(self.num_links):
                # transform each inertia matrix into joint space
                Mq_g += (J_links[ii].T * self._M_links[ii] * self.gravity)
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.num_joints):
                # transform each inertia matrix into joint space
                Mq_g += (J_joints[ii].T * self._M_joints[ii] * self.gravity)
            Mq_g = self.simplify(Mq_g)
            Mq_g = sp.Matrix(Mq_g)

            # save to file
            pickle.dump(Mq_g, open(
                '%s/Mq_g' % self.config_folder, 'wb'))

        if lambdify is False:
            return Mq_g
        if self.use_cython is True:
            return autowrap(Mq_g, backend="cython",
                            args=self.q)
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
            Tx = pickle.load(open('%s/%s.T' %
                                       (self.config_folder, filename), 'rb'))
        else:
            T = self._calc_T(name=name)
            # transform x into world coordinates
            if np.allclose(x, 0):
                # if we're only interested in the origin, not including
                # the x variables significantly speeds things up
                Tx = T * sp.Matrix([0, 0, 0, 1])
            else:
                # if we're interested in other points in the given frame
                # of reference, calculate transform with x variables
                Tx = T * sp.Matrix(self.x + [1])
            Tx = self.simplify(Tx)
            Tx = sp.Matrix(Tx)

            # save to file
            pickle.dump(sp.Matrix(Tx), open(
                '%s/%s.T' % (self.config_folder, filename), 'wb'))


        if lambdify is False:
            return Tx
        if self.use_cython is True:
            return autowrap(Tx, backend="cython",
                            args=self.q+self.x)
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
            T_inv = pickle.load(open('%s/%s.T_inv' %
                                          (self.config_folder,
                                           filename), 'rb'))
        else:
            T = self._calc_T(name=name)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]]))
            T_inv = sp.Matrix(T_inv)

            # save to file
            pickle.dump(T_inv, open(
                '%s/%s.T_inv' % (self.config_folder, filename), 'wb'))

        if lambdify is False:
            return T_inv
        if self.use_cython is True:
            return autowrap(T_inv, backend="cython",
                            args=self.q+self.x)
        return sp.lambdify(self.q + self.x, T_inv, "numpy")
