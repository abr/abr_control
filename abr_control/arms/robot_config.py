import cloudpickle
import hashlib
import importlib
import numpy as np
import os
import sympy as sp
from sympy.utilities.autowrap import autowrap
import sys

import abr_control
import abr_control.arms


# TODO : store lambdified functions, currently running into pickling errors
# cloudpickle, dill, and pickle all run into problems

class robot_config():
    """ Class defines a bunch of useful functions for controlling
    a given robot, including transformation to joints and COMs,
    Jacobians, the inertia matrix in joint space, and the effects
    of gravity. Uses SymPy and lambdify to do this.
    """

    def __init__(self, num_joints, num_links, robot_name="robot",
                 use_cython=False):
        """
        num_joints int: number of joints in robot
        num_links int: number of arm segments in robot
        robot_name string: used for saving/loading functions to file
        use_cython boolean: if True, a more efficient function is generated
                            useful when execution time is more important than
                            generation time
        """

        self.num_joints = num_joints
        self.num_links = num_links
        self.robot_name = robot_name

        self.use_cython = use_cython

        # create function dictionaries
        self._C = None  # placeholder for the centripetal and Coriolis function
        self._g = None  # placeholder for joint space gravity function
        self._dJ = {}  # for Jacobian time derivative calculations
        self._J = {}  # for Jacobian calculations
        self._M_links = []  # placeholder for (x,y,z) inertia matrices
        self._M_joints = []  # placeholder for (x,y,z) inertia matrices
        self._M = None  # placeholder for joint space inertia matrix function
        self._orientation = {}  # placeholder for orientation functions
        self._T_inv = {}  # for inverse transform calculations
        self._Tx = {}  # for point transform calculations
        self._R = {}  # for transform matrix calculations

        # specify / create the folder to save to and load from
        self.config_folder = (os.path.dirname(abr_control.arms.__file__) +
                              '/%s/saved_functions/' % robot_name)
        hasher = hashlib.md5()
        # TODO: make it so this file also affects hash
        with open(sys.modules[self.__module__].__file__, 'rb') as afile:
            buf = afile.read()
            hasher.update(buf)
        self.config_hash = hasher.hexdigest()
        self.config_folder += self.config_hash
        # make config folder if it doesn't exist
        abr_control.utils.os_utils.makedir(self.config_folder)

        # set up our joint angle symbols
        self.q = [sp.Symbol('q%i' % ii) for ii in range(self.num_joints)]
        self.dq = [sp.Symbol('dq%i' % ii) for ii in range(self.num_joints)]
        # set up an (x,y,z) offset
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]

        self.gravity = sp.Matrix([[0, 0, -9.81, 0, 0, 0]]).T

    def _generate_and_save_function(self, filename, expression, parameters):
        """ Create a folder in the saved_functions folder, based on a hash
        of the current robot_config subclass, save the functions created
        into this folder.

        If use_cython is True, specify a working directory for the autowrap
        function, so that binaries are saved inside and can be loaded in
        quickly later.
        """

        # check for / create the save folder for this expression
        folder = self.config_folder + '/' + filename
        abr_control.utils.os_utils.makedir(folder)

        if self.use_cython is True:
            # binaries saved by specifying tempdir parameter
            function = autowrap(expression, backend="cython",
                                args=parameters, tempdir=folder)
        function = sp.lambdify(parameters, expression, "numpy")

        return function

    def _load_from_file(self, filename, lambdify):
        """ Attempt to load in the specified function or expression from
        saved file, in a subfolder based on the hash of the robot_config

        filename string: the desired function to load in
        lambdify boolean: if True look for saved function
                          if False look for saved expression
        """
        expression = None
        function = None

        # check for / create the save folder for this expression
        folder = self.config_folder + '/' + filename
        if os.path.isdir(folder) is not False:
            # check to see should return function or expression
            if lambdify is True:
                if self.use_cython is True:
                    # check for cython binaries
                    saved_file = [sf for sf in os.listdir(folder)
                                  if sf.endswith('.so')]
                    if len(saved_file) > 0:
                        # if found, load in function from file
                        print('Loading cython function from %s ...' % filename)
                        if self.config_folder not in sys.path:
                            sys.path.append(self.config_folder)
                        saved_file = saved_file[0].split('.')[0]
                        function_binary = importlib.import_module(
                            filename + '.' + saved_file)
                        function = getattr(function_binary, 'autofunc_c')
                        # NOTE: This is a hack, but the above import command
                        # imports both 'filename.saved_file' and 'saved_file'
                        # having 'saved_file' in modules cause problems if
                        # the cython autofunc wrapper is used after this.
                        if saved_file in sys.modules.keys():
                            del sys.modules[saved_file]

            if function is None:
                # if function not loaded, check for saved expression
                if os.path.isfile('%s/%s/%s' %
                                  (self.config_folder, filename, filename)):
                    print('Loading expression from %s ...' % filename)
                    expression = cloudpickle.load(open(
                        '%s/%s/%s' % (self.config_folder, filename, filename),
                        'rb'))

        return expression, function

    def C(self, q, dq):
        """ Calculates the centripetal and Coriolis forces

        q list: (radians) joint angles
        dq list: (radians/sec) joint velocities
        """
        # check for function in dictionary
        if self._C is None:
            self._C = self._calc_C()
        parameters = tuple(q) + tuple(dq)
        return np.array(self._C(*parameters), dtype='float32').flatten()

    def g(self, q):
        """ Calculates the force of gravity in joint space for the ur5

        q list: (radians) joint angles
        """
        # check for function in dictionary
        if self._g is None:
            self._g = self._calc_g()
        parameters = tuple(q)
        return np.array(self._g(*parameters), dtype='float32').flatten()

    def dJ(self, name, q, dq, x=[0, 0, 0]):
        """ Calculates the derivative of a Jacobian for a joint or link
        with respect to time

        name string: name of the joint or link, or end-effector
        q list: (radians) joint angles
        x list: (m) the [x,y,z] offset inside name's reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._dJ.get(funcname, None) is None:
            self._dJ[funcname] = self._calc_dJ(name=name, x=x)
        parameters = tuple(q) + tuple(dq) + tuple(x)
        return np.array(self._dJ[funcname](*parameters), dtype='float32')

    def J(self, name, q, x=[0, 0, 0]):
        """ Calculates the Jacobian for a joint or link

        name string: name of the joint or link, or end-effector
        q list: (radians) joint angles
        x list: (m) the [x,y,z] offset inside name's reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._J.get(funcname, None) is None:
            self._J[funcname] = self._calc_J(name=name, x=x)
        parameters = tuple(q) + tuple(x)
        return np.array(self._J[funcname](*parameters), dtype='float32')

    def M(self, q):
        """ Calculates the joint space inertia matrix for the ur5

        q list: (radians) joint angles
        """
        # check for function in dictionary
        if self._M is None:
            self._M = self._calc_M()
        parameters = tuple(q)
        return np.array(self._M(*parameters), dtype='float32')

    def orientation(self, name, q):
        """ Uses Sympy to generate the orientation for a joint or link
        calculated as a quaternion.

        name string: name of the joint or link, or end-effector
        q list: (radians) joint angles
        """
        # check for function in dictionary
        if self._R.get(name, None) is None:
            self._R[name] = self._calc_R(name)
        parameters = tuple(q)

        R = self._R[name](*parameters)
        return abr_control.utils.transformations.quaternion_from_matrix(R)

    def Tx(self, name, q, x=[0, 0, 0]):
        """ Calculates the transform for a joint or link

        name string: name of the joint or link, or end-effector
        q list: (radians) joint angles
        x list: (m) the [x,y,z] offset inside name's reference frame
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._Tx.get(funcname, None) is None:
            self._Tx[funcname] = self._calc_Tx(name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._Tx[funcname](*parameters)[:-1].flatten()

    def T_inv(self, name, q, x=[0, 0, 0]):
        """ Calculates the inverse transform for a joint or link

        q list: (radians) joint angles
        """
        funcname = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._T_inv.get(funcname, None) is None:
            self._T_inv[funcname] = self._calc_T_inv(name=name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._T_inv[funcname](*parameters)

    def _calc_C(self, lambdify=True):
        """ Uses Sympy to generate the centripetal and Coriolis forces
        affecting the arm

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        C = None
        C_func = None
        # check to see if we have our gravity term saved in file
        C, C_func = self._load_from_file('C', lambdify)

        if C is None and C_func is None:
            # if no saved file was loaded, generate function
            print('Generating centripetal and Coriolis compensation function')

            # first get the inertia matrix
            M = self._calc_M(lambdify=False)
            # http://www.diag.uniroma1.it/~deluca/rob2_en/03_LagrangianDynamics_1.pdf
            # c_k = dq.T * C_k * dq
            # C_k = .5 * (\frac{\partial m_k}{\partial q} +
            #           \frac{\partial m_k}{\partial q}^T +
            #           \frac{\partial M}{\partia q_k})
            # where c_k and m_k are the kth element of C and column of M
            C = sp.zeros(6, 1)
            for kk in range(6):
                dMkdq = M[:, kk].jacobian(sp.Matrix(self.q))
                Ck = 0.5 * (dMkdq + dMkdq.T - M.diff(self.q[kk]))
                C[kk] = sp.Matrix(self.dq).T * Ck * sp.Matrix(self.dq)
            C = sp.Matrix(C)

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/C' % self.config_folder)
            cloudpickle.dump(C, open(
                '%s/C/C' % self.config_folder, 'wb'))

        if lambdify is False:
            # if should return expression not function
            return C

        if C_func is None:
            C_func = self._generate_and_save_function(
                filename='C', expression=C,
                parameters=self.q+self.dq)
        return C_func

    def _calc_g(self, lambdify=True):
        """ Uses Sympy to generate the force of gravity in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        g = None
        g_func = None
        # check to see if we have our gravity term saved in file
        g, g_func = self._load_from_file('g', lambdify)

        if g is None and g_func is None:
            # if no saved file was loaded, generate function
            print('Generating gravity compensation function')

            # get the Jacobians for each link's COM
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0],
                                    lambdify=False)
                       for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0],
                                     lambdify=False)
                        for ii in range(self.num_joints)]

            # sum together the effects of each arm segment's inertia
            g = sp.zeros(self.num_joints, 1)
            for ii in range(self.num_links):
                # transform each inertia matrix into joint space
                g += (J_links[ii].T * self._M_links[ii] * self.gravity)
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.num_joints):
                # transform each inertia matrix into joint space
                g += (J_joints[ii].T * self._M_joints[ii] * self.gravity)
            g = sp.Matrix(g)

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/g' % self.config_folder)
            cloudpickle.dump(g, open(
                '%s/g/g' % self.config_folder, 'wb'))

        if lambdify is False:
            # if should return expression not function
            return g

        if g_func is None:
            g_func = self._generate_and_save_function(
                filename='g', expression=g,
                parameters=self.q)
        return g_func


    def _calc_dJ(self, name, x, lambdify=True):
        """ Uses Sympy to generate the derivative of the Jacobian
        for a joint or link with respect to time

        name string: name of the joint or link, or end-effector
        x list: (m) the [x,y,z] offset inside name's reference frame
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        dJ = None
        dJ_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_dJ'
        # check to see if should try to load functions from file
        dJ, dJ_func = self._load_from_file(filename, lambdify)

        if dJ is None and dJ_func is None:
            # if no saved file was loaded, generate function
            print('Generating derivative of Jacobian ',
                  'function for %s' % filename)

            J = self._calc_J(name, x=x, lambdify=False)
            dJ = sp.Matrix(np.zeros(J.shape, dtype='float32'))
            # calculate derivative of (x,y,z) wrt to time
            # which each joint is dependent on
            for ii in range(J.shape[0]):
                for jj in range(J.shape[1]):
                    for kk in range(self.num_joints):
                        dJ[ii, jj] += J[ii, jj].diff(self.q[kk]) * self.dq[kk]
            dJ = sp.Matrix(dJ)

            # save expression to file
            abr_control.utils.os_utils.makedir(
                '%s/%s' % (self.config_folder, filename))
            cloudpickle.dump(dJ, open(
                '%s/%s/%s' % (self.config_folder, filename, filename), 'wb'))

        if lambdify is False:
            # if should return expression not function
            return dJ

        if dJ_func is None:
            dJ_func = self._generate_and_save_function(
                filename=filename, expression=dJ,
                parameters=self.q+self.dq+self.x)
        return dJ_func

    def _calc_J(self, name, x, lambdify=True):
        """ Uses Sympy to generate the Jacobian for a joint or link

        name string: name of the joint or link, or end-effector
        x list: (m) the [x,y,z] offset inside name's reference frame
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        J = None
        J_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_J'

        # check to see if should try to load functions from file
        J, J_func = self._load_from_file(filename, lambdify)

        if J is None and J_func is None:
            # if no saved file was loaded, generate function
            print('Generating Jacobian function for %s' % filename)

            Tx = self._calc_Tx(name, x=x, lambdify=False)
            # NOTE: calculating the Jacobian this way doesn't incur any
            # real computational cost (maybe 30ms) and it simplifies adding
            # the orientation information below (as opposed to using
            # sympy's Tx.jacobian method)
            J = []
            # calculate derivative of (x,y,z) wrt to each joint
            for ii in range(self.num_joints):
                J.append([])
                J[ii].append(Tx[0].diff(self.q[ii]))  # dx/dq[ii]
                J[ii].append(Tx[1].diff(self.q[ii]))  # dy/dq[ii]
                J[ii].append(Tx[2].diff(self.q[ii]))  # dz/dq[ii]

            end_point = name.strip('link').strip('joint')
            end_point = self.num_joints if 'EE' in end_point else end_point

            end_point = min(int(end_point) + 1, self.num_joints)
            # add on the orientation information up to the last joint
            for ii in range(end_point):
                J[ii] = J[ii] + list(self.J_orientation[ii])
            # fill in the rest of the joints orientation info with 0
            for ii in range(end_point, self.num_joints):
                J[ii] = J[ii] + [0, 0, 0]
            J = sp.Matrix(J).T  # correct the orientation of J

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/%s' % (self.config_folder, filename))
            cloudpickle.dump(J, open(
                '%s/%s/%s' % (self.config_folder, filename, filename), 'wb'))

        if lambdify is False:
            # if should return expression not function
            return J

        if J_func is None:
            J_func = self._generate_and_save_function(
                filename=filename, expression=J,
                parameters=self.q+self.x)
        return J_func

    def _calc_M(self, lambdify=True):
        """ Uses Sympy to generate the inertia matrix in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """

        M = None
        M_func = None

        # check to see if we have our inertia matrix saved in file
        M, M_func = self._load_from_file('M', lambdify)

        if M is None and M_func is None:
            # if no saved file was loaded, generate function
            print('Generating inertia matrix function')

            # get the Jacobians for each link's COM
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0],
                                    lambdify=False)
                       for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0],
                                     lambdify=False)
                        for ii in range(self.num_joints)]

            # sum together the effects of each arm segment's inertia
            M = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                # transform each inertia matrix into joint space
                M += (J_links[ii].T * self._M_links[ii] * J_links[ii])
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.num_joints):
                # transform each inertia matrix into joint space
                M += (J_joints[ii].T * self._M_joints[ii] * J_joints[ii])
            M = sp.Matrix(M)

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/M' % (self.config_folder))
            cloudpickle.dump(M, open(
                '%s/M/M' % self.config_folder, 'wb'))

        if lambdify is False:
            # if should return expression not function
            return M

        if M_func is None:
            M_func = self._generate_and_save_function(
                filename='M', expression=M,
                parameters=self.q)
        return M_func

    def _calc_R(self, name, lambdify=True):
        """ Uses Sympy to generate the rotation matrix for a joint or link

        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        R = None
        R_func = None
        filename = name + '_R'

        # check to see if we have the rotation matrix saved in file
        R, R_func = self._load_from_file(filename, lambdify=True)

        if R is None and R_func is None:
            # if no saved file was loaded, generate function
            print('Generating rotation matrix function.')
            R = self._calc_T(name=name)[:3, :3]

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/%s' % (self.config_folder, filename))
            cloudpickle.dump(sp.Matrix(R), open(
                '%s/%s/%s' % (self.config_folder, filename, filename),
                'wb'))

        if R_func is None:
            R_func = self._generate_and_save_function(
                filename=filename, expression=R,
                parameters=self.q)
        return R_func

    def _calc_T(self, name):
        """ Uses Sympy to generate the transform for a joint or link

        name string: name of the joint or link, or end-effector
        """
        raise NotImplementedError("_calc_T function not implemented")

    def _calc_Tx(self, name, x=None, lambdify=True):
        """ Uses Sympy to transform x from the reference frame of a joint
        or link to the origin (world) coordinates.

        name string: name of the joint or link, or end-effector
        x list: (m) the [x,y,z] offset inside name's reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        Tx = None
        Tx_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_Tx'
        # check to see if we have our transformation saved in file
        Tx, Tx_func = self._load_from_file(filename, lambdify)

        if Tx is None and Tx_func is None:
            print('Generating transform function for %s' % filename)
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
            Tx = sp.Matrix(Tx)

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/%s' % (self.config_folder, filename))
            cloudpickle.dump(sp.Matrix(Tx), open(
                '%s/%s/%s.Tx' % (self.config_folder, filename, filename),
                'wb'))

        if lambdify is False:
            # if should return expression not function
            return Tx

        if Tx_func is None:
            Tx_func = self._generate_and_save_function(
                filename=filename, expression=Tx,
                parameters=self.q+self.x)
        return Tx_func

    def _calc_T_inv(self, name, x, lambdify=True):
        """ Return the inverse transform matrix, which converts from
        world coordinates into the robot's end-effector reference frame

        name string: name of the joint or link, or end-effector
        x list: (m) the [x,y,z] offset inside name's reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        T_inv = None
        T_inv_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_Tinv'
        # check to see if we have our transformation saved in file
        T_inv, T_inv_func = self._load_from_file(filename, lambdify)

        if T_inv is None and T_inv_func is None:
            print('Generating inverse transform function for %s' % filename)
            T = self._calc_T(name=name)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]]))
            T_inv = sp.Matrix(T_inv)

            # save to file
            abr_control.utils.os_utils.makedir(
                '%s/%s' % (self.config_folder, filename))
            cloudpickle.dump(T_inv, open(
                '%s/%s.T_inv' % (self.config_folder, filename), 'wb'))

        if lambdify is False:
            # if should return expression not function
            return T_inv

        if T_inv_func is None:
            T_inv_func = self._generate_and_save_function(
                filename=filename, expression=T_inv,
                parameters=self.q+self.x)
        return T_inv_func
