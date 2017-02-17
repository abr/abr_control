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


# TODO : store both the lambdified and unlambdified versions of everything

# TODO : check to see how long it takes to load T from file vs recompute

class robot_config():
    """ Class defines a bunch of useful functions for controlling
    a given robot, including transformation to joints and COMs,
    Jacobians, the inertia matrix in joint space, and the effects
    of gravity. Uses SymPy and lambdify to do this.
    """

    def __init__(self, num_joints, num_links, robot_name="robot",
                 regenerate_functions=False, use_cython=False):
        """
        num_joints int: number of joints in robot
        num_links int: number of arm segments in robot
        robot_name string: used for saving/loading functions to file
        regenerate_functions boolean: if True, don't look to saved files
                                      regenerate all transforms and Jacobians
        use_cython boolean: if True, a more efficient function is generated
                            useful when execution time is more important than
                            generation time
        """

        self.num_joints = num_joints
        self.num_links = num_links
        self.robot_name = robot_name

        self.regenerate_functions = regenerate_functions
        self.use_cython = use_cython

        # create function dictionaries
        self._Tx = {}  # for transform calculations
        self._T_inv = {}  # for inverse transform calculations
        self._J = {}  # for Jacobian calculations
        self._M_links = []  # placeholder for (x,y,z) inertia matrices
        self._M_joints = []  # placeholder for (x,y,z) inertia matrices
        self._Mq = None  # placeholder for joint space inertia matrix function
        self._g = None  # placeholder for joint space gravity term function
        self._orientation = {}  # placeholder for orientation functions
        self._T_inv = {}  # for inverse transform calculations
        self._Tx = {}  # for point transform calculations
        self._T_func = {}  # for transform matrix calculations

        # specify / create the folder to save to and load from
        self.config_folder = (os.path.dirname(abr_control.arms.__file__) +
                              '/%s/saved_functions/' % robot_name)
        hasher = hashlib.md5()
        with open(sys.modules[self.__module__].__file__, 'rb') as afile:
            buf = afile.read()
            hasher.update(buf)
        self.config_folder += hasher.hexdigest()
        # make config folder if it doesn't exist
        abr_control.utils.os.makedir(self.config_folder)

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
            self._Mq = self._calc_Mq(regenerate=self.regenerate_functions)
        parameters = tuple(q)
        return np.array(self._Mq(*parameters), dtype='float32')

    def g(self, q):
        """ Calculates the force of gravity in joint space for the ur5

        q list: set of joint angles to pass in to the g function
        """
        # check for function in dictionary
        if self._g is None:
            self._g = self._calc_g(regenerate=self.regenerate_functions)
        parameters = tuple(q)
        return np.array(self._g(*parameters), dtype='float32').flatten()

    def orientation(self, name, q):
        """ Uses Sympy to generate the orientation for a joint or link
        calculated as a quaternion.

        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the g function
        """
        # get transform matrix for reference frame of interest
        if self._T_func.get(name, None) is None:
            print('Generating orientation function.')
            # check to see if we have our transformation saved in file
            if (self.regenerate_functions is False and
                    os.path.isfile('%s/%s.T' % (self.config_folder, name))):
                T = cloudpickle.load(open('%s/%s.T' %
                                     (self.config_folder, name),
                                     'rb'))
            else:
                T = self._calc_T(name=name)

                # save to file
                cloudpickle.dump(sp.Matrix(T), open(
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
            self._T_inv[funcname] = self._calc_T_inv(
                name=name, x=x, regenerate=self.regenerate_functions)
        parameters = tuple(q) + tuple(x)
        return self._T_inv[funcname](*parameters)

    def _generate_and_save_function(self, filename, expression, parameters):
        """ Create a folder in the saved_functions folder, based on a hash
        of the current robot_config subclass.

        If use_cython is True, specify a working directory for the autowrap
        function, so that binaries are saved inside and can be loaded in
        quickly later.
        """

        # check for / create the save folder for this expression
        folder = self.config_folder + '/' + filename
        abr_control.utils.os.makedir(folder)

        if self.use_cython is True:
            # binaries saved by specifying tempdir parameter
            function = autowrap(expression, backend="cython",
                                args=parameters, tempdir=folder)
        else:
            function = sp.lambdify(parameters, expression, "numpy")
            # save function to file
            cloudpickle.dump(function, open(
                '%s/%s.func' % (folder, filename),
                'wb'))

        return function

    def _load_from_file(self, filename, lambdify):
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
                        # sys.path.append(folder)
                        if self.config_folder not in sys.path:
                            sys.path.append(self.config_folder)
                        saved_file = saved_file[0].split('.')[0]
                        function_binary = importlib.import_module(
                            filename + '.' + saved_file)
                        function = getattr(function_binary, 'autofunc_c')
                        # sys.path.remove(folder)
                        # sys.path.remove(self.config_folder)
                else:
                    # check for saved function
                    extension = '.func'
                    full_filename = '%s/%s%s' % (self.config_folder,
                                                 filename, extension)

                    if os.path.isfile(full_filename):
                        print('Loading function from %s%s ...' % (filename,
                                                                  extension))
                        function = cloudpickle.load(open(full_filename, 'rb'))

            if function is None:
                # if function not loaded, check for saved expression
                if os.path.isfile('%s/%s' %
                                  (self.config_folder, filename)):
                    print('Loading expression from %s ...' % filename)
                    expression = cloudpickle.load(open(
                        '%s/%s' % (self.config_folder, filename), 'rb'))

        return expression, function

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

        dJ = None
        dJ_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_dJ'
        # check to see if should try to load functions from file
        if regenerate is False:
            dJ, dJ_func = self._load_from_file(filename, lambdify)

        if dJ is None and dJ_func is None:
            # if no saved file was loaded, generate function
            print('Generating derivative of Jacobian ',
                  'function for %s' % filename)

            # TODO: make sure that we're not regenerating all these
            # Jacobians again here if they've already been regenerated
            # once, but they are actually regenerated if only this is called
            J = self._calc_J(name, x=x, lambdify=False, regenerate=regenerate)
            dJ = sp.Matrix(np.zeros(J.shape, dtype='float32'))
            # calculate derivative of (x,y,z) wrt to time
            # which each joint is dependent on
            for ii in range(J.shape[0]):
                for jj in range(J.shape[1]):
                    for kk in range(self.num_joints):
                        dJ[ii, jj] += J[ii, jj].diff(self.q[kk])
            dJ = sp.Matrix(dJ).T

            # save expression to file
            cloudpickle.dump(dJ, open(
                '%s/%s' % (self.config_folder, filename), 'wb'))

        if lambdify is False:
            # if should return expression not function
            return dJ
        return self._generate_and_save_function(
            filename=filename, expression=dJ,
            parameters=self.q+self.x)

    def _calc_J(self, name, x, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the Jacobian for a joint or link

        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """
        J = None
        J_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        filename += '_J'

        # check to see if should try to load functions from file
        if regenerate is False:
            J, J_func = self._load_from_file(filename, lambdify)

        if J is None and J_func is None:
            # if no saved file was loaded, generate function
            print('Generating Jacobian function for %s' % filename)

            # TODO: make sure that we're not regenerating all these
            # Transforms again here if they've already been regenerated
            # once, but they are actually regenerated if only this is called
            Tx = self._calc_Tx(name, x=x, lambdify=False,
                               regenerate=regenerate)
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
            abr_control.utils.os.makedir(
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

    def _calc_Mq(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the inertia matrix in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        Mq = None
        Mq_func = None

        # check to see if we have our inertia matrix saved in file
        if regenerate is False:
            Mq, Mq_func = self._load_from_file('Mq', lambdify)

        if Mq is None and Mq_func is None:
            # if no saved file was loaded, generate function
            print('Generating inertia matrix function')

            # get the Jacobians for each link's COM
            # TODO: make sure that we're not regenerating all these
            # Jacobians again here if they've already been regenerated
            # once, but they are actually regenerated if only this is called
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0],
                                    lambdify=False, regenerate=regenerate)
                       for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0],
                                     lambdify=False, regenerate=regenerate)
                        for ii in range(self.num_joints)]

            # sum together the effects of each arm segment's inertia
            Mq = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                # transform each inertia matrix into joint space
                Mq += (J_links[ii].T * self._M_links[ii] * J_links[ii])
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.num_joints):
                # transform each inertia matrix into joint space
                Mq += (J_joints[ii].T * self._M_joints[ii] * J_joints[ii])
            Mq = sp.Matrix(Mq)

            # save to file
            abr_control.utils.os.makedir(
                '%s/Mq' % (self.config_folder))
            cloudpickle.dump(Mq, open(
                '%s/Mq/Mq' % self.config_folder, 'wb'))

        if lambdify is False:
            # if should return expression not function
            return Mq

        if Mq_func is None:
            Mq_func = self._generate_and_save_function(
                filename='Mq', expression=Mq,
                parameters=self.q)
        return Mq_func

    def _calc_g(self, lambdify=True, regenerate=False):
        """ Uses Sympy to generate the force of gravity in
        joint space

        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        regenerate boolean: if True, don't use saved functions
        """

        g = None
        g_func = None
        # check to see if we have our gravity term saved in file
        if regenerate is False:
            g, g_func = self._load_from_file('g', lambdify)

        if g is None and g_func is None:
            # if no saved file was loaded, generate function
            print('Generating gravity compensation function')

            # get the Jacobians for each link's COM
            # TODO: make sure that we're not regenerating all these
            # Jacobians again here if they've already been regenerated
            # once, but they are actually regenerated if only this is called
            J_links = [self._calc_J('link%s' % ii, x=[0, 0, 0],
                                    lambdify=False, regenerate=regenerate)
                       for ii in range(self.num_links)]
            J_joints = [self._calc_J('joint%s' % ii, x=[0, 0, 0],
                                     lambdify=False, regenerate=regenerate)
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
            abr_control.utils.os.makedir(
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

        Tx = None
        Tx_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our transformation saved in file
        if regenerate is False:
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
            abr_control.utils.os.makedir(
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

        T_inv = None
        T_inv_func = None
        filename = name + '[0,0,0]' if np.allclose(x, 0) else name
        # check to see if we have our transformation saved in file
        if regenerate is False:
            T_inv, T_inv_func = self._load_from_file(filename, lambdify)

        if T_inv is None and T_inv_func is None:
            print('Generating inverse transform function for %s' % filename)
            T = self._calc_T(name=name, regenerate=regenerate)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]]))
            T_inv = sp.Matrix(T_inv)

            # save to file
            abr_control.utils.os.makedir(
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


