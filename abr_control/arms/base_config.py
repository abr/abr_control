import hashlib
import importlib
import os
import sys

import numpy as np

import cloudpickle

import sympy as sp
from sympy.utilities.autowrap import autowrap

import abr_control.utils.os_utils
from abr_control.utils.paths import cache_dir
from abr_control.utils import transformations


# TODO : store lambdified functions, currently running into pickling errors
# cloudpickle, dill, and pickle all run into problems


class BaseConfig:
    """
    Defines useful functions for controlling a robot

    Creates functions for calculating transformation to joints and COMs,
    Jacobians, the inertia matrix in joint space, and the effects
    of gravity. Uses SymPy and lambdify to do this.

    In the _calc_* methods, setting lambdify = True will return a
    function to calculate the matrix being generated. If the user
    desires a symbolic expression, call the _calc_* methods with
    lambdify = False. Lambdify is True by default.

    Parameters
    ----------
    N_JOINTS : int
        number of joints in robot
    N_LINKS : int
        number of arm segments in robot
    ROBOT_NAME : string, optional (Default: "robot")
        used for saving/loading functions to file
    use_cython : boolean, optional (Default: False)
        if True, a more efficient function is generated
        useful when execution time is more important than
        generation time

    Attributes
    ----------
        _C : function
            placeholder for the partial centrifugal and Coriolis function
        _dJ : dictionary
            for Jacobian time derivative functions of joints and COMs
        _g : function
            placeholder for joint space gravity function
        _J  : dictionary
            for Jacobian calculations
        _KZ : sympy.Matrix
            z isolation vector for calculating orientation part of Jacobian
        _M_LINKS : list
            inertia matrices of the robot links
        _M_JOINTS : list
            inertia matrices of the robot joints
        _M : function
            placeholder for joint space inertia matrix function
        _orientation : dictionary
            placeholder for orientation functions of joints and COMs
        _R : dictionary
            for transform matrix calculations for joints and COMs
        _T_inv : dictionary
            for inverse transform calculations for joints and COMs
        _Tx : dictionary
            for point transform calculations for joints and COMs
        _T : dictionary
            for transformation matrices for joints and COMs
        config_folder : string
            location to save to and load functions from, based on the hash
            of the subclass, so that generated functions are saved uniquely
    """

    def __init__(self, N_JOINTS, N_LINKS, ROBOT_NAME="robot", use_cython=True):

        self.N_JOINTS = N_JOINTS
        self.N_LINKS = N_LINKS
        self.ROBOT_NAME = ROBOT_NAME
        self.use_cython = use_cython

        # create function placeholders and dictionaries
        self._C = None
        self._dJ = {}
        self._g = None
        self._J = {}
        self._M = None
        self._orientation = {}
        self._R = {}
        self._T_inv = {}
        self._Tx = {}
        self._T_func = {}

        self._KZ = sp.Matrix([0, 0, 1])

        # inertia matrix lists, to be filled out by subclasses
        self._M_LINKS = []
        self._M_JOINTS = []

        # specify / create the folder to save to and load from
        self.config_folder = cache_dir + "/%s/saved_functions/" % ROBOT_NAME
        # create a unique hash for the config file
        hasher = hashlib.md5()
        with open(sys.modules[self.__module__].__file__, "rb") as afile:
            buf = afile.read()
            hasher.update(buf)
        self.config_hash = hasher.hexdigest()
        self.config_folder += self.config_hash
        # make config folder if it doesn't exist
        abr_control.utils.os_utils.makedirs(self.config_folder)

        # set up our joint angle symbols
        self.q = [sp.Symbol("q%i" % ii) for ii in range(self.N_JOINTS)]
        self.dq = [sp.Symbol("dq%i" % ii) for ii in range(self.N_JOINTS)]
        # set up an (x,y,z) offset
        self.x = [sp.Symbol("x"), sp.Symbol("y"), sp.Symbol("z")]

        self.x_zeros = np.zeros(3)

        self.gravity = sp.Matrix([[0, 0, -9.81, 0, 0, 0]]).T

    def _generate_and_save_function(self, filename, expression, parameters):
        """ Creates a folder, saves generated cython functions

        Create a folder in the users cache directory, named based on a hash
        of the current robot_config subclass.

        If use_cython is True, uses the created folder to save the autowrap
        binaries, so that they can be loaded in quickly later.
        """

        # check for / create the save folder for this expression
        folder = self.config_folder + "/" + filename
        abr_control.utils.os_utils.makedirs(folder)

        if self.use_cython is True:
            # binaries saved by specifying tempdir parameter
            function = autowrap(
                expression, backend="cython", args=parameters, tempdir=folder
            )
        function = sp.lambdify(parameters, expression, "numpy")

        return function

    def _load_from_file(self, filename, lambdify):
        """ Attempts to load in saved files

        Attempt to load in the specified function or expression from
        saved file, in a subfolder based on the hash of the robot_config.
        Takes a filename as an input and returns the function or expression,
        depending on if lambdify is True or False, respectively.

        Parameters
        ----------
        filename : string
            the desired function to load in
        lambdify : boolean
            if True returns (None, function to calculate the matrix)
            if False returns (the Sympy matrix, None)
        """

        expression = None
        function = None

        # check for / create the save folder for this expression
        folder = self.config_folder + "/" + filename
        if os.path.isdir(folder) is not False:
            # check to see should return function or expression
            if lambdify is True:
                if self.use_cython is True:
                    # check for cython binaries
                    saved_file = [sf for sf in os.listdir(folder) if sf.endswith(".so")]
                    if len(saved_file) > 0:
                        # if found, load in function from file
                        if self.config_folder not in sys.path:
                            sys.path.append(self.config_folder)
                        saved_file = saved_file[0].split(".")[0]
                        function_binary = importlib.import_module(
                            filename + "." + saved_file
                        )
                        function = getattr(function_binary, "autofunc_c")
                        # NOTE: This is a hack, but the above import command
                        # imports both 'filename.saved_file' and 'saved_file'.
                        # Having 'saved_file' in modules cause problems if
                        # the cython autofunc wrapper is used after this.
                        if saved_file in sys.modules.keys():
                            del sys.modules[saved_file]

            if function is None:
                # if function not loaded, check for saved expression
                if os.path.isfile(
                    "%s/%s/%s" % (self.config_folder, filename, filename)
                ):
                    print("Loading expression from %s ..." % filename)
                    expression = cloudpickle.load(
                        open(
                            "%s/%s/%s" % (self.config_folder, filename, filename), "rb"
                        )
                    )

        return expression, function

    def g(self, q):
        """ Loads or calculates the force of gravity in joint space

        Parameters
        ----------
        q : numpy.array
            joint angles [radians]

        """
        # check for function in dictionary
        if self._g is None:
            self._g = self._calc_g()
        parameters = tuple(q)
        return np.array(self._g(*parameters), dtype="float32").flatten()

    def dJ(self, name, q, dq, x=None):
        """ Loads or calculates the derivative of the Jacobian wrt time

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        x : numpy.array, optional (Default: None)
            the [x,y,z] offset inside reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.

        """

        x = self.x_zeros if x is None else x
        funcname = name + "[0,0,0]" if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._dJ.get(funcname, None) is None:
            self._dJ[funcname] = self._calc_dJ(name=name, x=x)
        parameters = tuple(q) + tuple(dq) + tuple(x)
        return np.array(self._dJ[funcname](*parameters), dtype="float32")

    def J(self, name, q, x=None):
        """ Loads or calculates the Jacobian for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        x : numpy.array, optional (Default: [0,0,0])
            the [x,y,z] offset inside reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        """

        x = self.x_zeros if x is None else x
        funcname = name + "[0,0,0]" if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._J.get(funcname, None) is None:
            self._J[funcname] = self._calc_J(name=name, x=x)
        parameters = tuple(q) + tuple(x)
        return np.array(self._J[funcname](*parameters), dtype="float32")

    def M(self, q):
        """ Loads or calculates the joint space inertia matrix

        Parameters
        ----------
        q : numpy.array
            joint angles [radians]
        """

        # check for function in dictionary
        if self._M is None:
            self._M = self._calc_M()
        parameters = tuple(q)
        return np.array(self._M(*parameters), dtype="float32")

    def R(self, name, q):
        """ Loads or calculates the rotation matrix

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        """
        # check for function in dictionary
        if self._R.get(name, None) is None:
            self._R[name] = self._calc_R(name)
        parameters = tuple(q)
        return np.array(self._R[name](*parameters), dtype="float32")

    # TODO: optimize this function
    def quaternion(self, name, q):
        """ Gets orientation matrix and converts to a quaternion

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        """
        R = self.R(name=name, q=q)
        quat = transformations.unit_vector(
            transformations.quaternion_from_matrix(matrix=R)
        )
        return quat

    def C(self, q, dq):
        """ Loads or calculates the centrifugal and Coriolis forces matrix
        such that np.dot(C, dq) is the full term

        Parameters
        ----------
        q : numpy.array
            joint angles [radians]
        dq : numpy.array
            joint velocities [radians/second]

        """
        # check for function in dictionary
        if self._C is None:
            self._C = self._calc_C()
        parameters = tuple(q) + tuple(dq)
        return np.array(self._C(*parameters), dtype="float32")

    def T(self, name, q):
        """ Loads or calculates the transformation Matrix for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        # check for function in dictionary
        if self._T_func.get(name, None) is None:
            filename = name + "_T"
            T, T_func = self._load_from_file(filename, lambdify=True)

            if T is None and T_func is None:
                T = self._calc_T(name)

                # save to file
                abr_control.utils.os_utils.makedirs(
                    "%s/%s" % (self.config_folder, filename)
                )
                cloudpickle.dump(
                    T, open("%s/%s/%s" % (self.config_folder, filename, filename), "wb")
                )

            if T_func is None:
                T_func = self._generate_and_save_function(
                    filename=filename, expression=T, parameters=self.q
                )

            self._T_func[name] = T_func

        parameters = tuple(q)
        return self._T_func[name](*parameters)

    def Tx(self, name, q, x=None):
        """ Loads or calculates the (x, y, z) position for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        x : numpy.array, optional (Default: [0,0,0])
            the [x,y,z] offset inside reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        """

        x = self.x_zeros if x is None else x
        funcname = name + "[0,0,0]" if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._Tx.get(funcname, None) is None:
            self._Tx[funcname] = self._calc_Tx(name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._Tx[funcname](*parameters)[:-1].flatten()

    def T_inv(self, name, q, x=None):
        """ Loads or calculates the inverse transform for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        q : numpy.array
            joint angles [radians]
        x : numpy.array, optional (Default: [0,0,0])
            the [x,y,z] offset inside reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        """

        x = self.x_zeros if x is None else x
        funcname = name + "[0,0,0]" if np.allclose(x, 0) else name
        # check for function in dictionary
        if self._T_inv.get(funcname, None) is None:
            self._T_inv[funcname] = self._calc_T_inv(name=name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._T_inv[funcname](*parameters)

    def _calc_g(self, lambdify=True):
        """ Generate the force of gravity in joint space

        Uses Sympy to generate the force of gravity in joint space

        Parameters
        ----------
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """
        g = None
        g_func = None
        # check to see if we have our gravity term saved in file
        g, g_func = self._load_from_file("g", lambdify)

        if g is None and g_func is None:
            # if no saved file was loaded, generate function
            print("Generating gravity compensation function")

            # get the Jacobians for each link's COM
            J_links = [
                self._calc_J("link%s" % ii, x=self.x_zeros, lambdify=False)
                for ii in range(self.N_LINKS)
            ]
            J_joints = [
                self._calc_J("joint%s" % ii, x=self.x_zeros, lambdify=False)
                for ii in range(self.N_JOINTS)
            ]

            # sum together the effects of each arm segment's inertia
            g = sp.zeros(self.N_JOINTS, 1)
            for ii in range(self.N_LINKS):
                # transform each inertia matrix into joint space
                g += J_links[ii].T * self._M_LINKS[ii] * self.gravity
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.N_JOINTS):
                # transform each inertia matrix into joint space
                g += J_joints[ii].T * self._M_JOINTS[ii] * self.gravity
            g = sp.Matrix(g)

            # save to file
            abr_control.utils.os_utils.makedirs("%s/g" % self.config_folder)
            cloudpickle.dump(g, open("%s/g/g" % self.config_folder, "wb"))

        if lambdify is False:
            # if should return expression not function
            return g

        if g_func is None:
            g_func = self._generate_and_save_function(
                filename="g", expression=g, parameters=self.q
            )
        return g_func

    def _calc_dJ(self, name, x, lambdify=True):
        """ Generate the derivative of the Jacobian

        Uses Sympy to generate the derivative of the Jacobian
        for a joint or link with respect to time

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        x : numpy.array
            the [x,y,z] offset inside the reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        dJ = None
        dJ_func = None
        filename = name + "[0,0,0]" if np.allclose(x, 0) else name
        filename += "_dJ"
        # check to see if should try to load functions from file
        dJ, dJ_func = self._load_from_file(filename, lambdify)

        if dJ is None and dJ_func is None:
            # if no saved file was loaded, generate function
            print("Generating derivative of Jacobian ", "function for %s" % filename)

            J = self._calc_J(name, x=x, lambdify=False)
            dJ = sp.Matrix(np.zeros(J.shape, dtype="float32"))
            # calculate derivative of (x,y,z) wrt to time
            # which each joint is dependent on
            for ii in range(J.shape[0]):
                for jj in range(J.shape[1]):
                    for kk in range(self.N_JOINTS):
                        dJ[ii, jj] += J[ii, jj].diff(self.q[kk]) * self.dq[kk]
            dJ = sp.Matrix(dJ)

            # save expression to file
            abr_control.utils.os_utils.makedirs(
                "%s/%s" % (self.config_folder, filename)
            )
            cloudpickle.dump(
                dJ, open("%s/%s/%s" % (self.config_folder, filename, filename), "wb")
            )

        if lambdify is False:
            # if should return expression not function
            return dJ

        if dJ_func is None:
            dJ_func = self._generate_and_save_function(
                filename=filename, expression=dJ, parameters=self.q + self.dq + self.x
            )
        return dJ_func

    def _calc_J(self, name, x, lambdify=True):
        """ Uses Sympy to generate the Jacobian for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        x : numpy.array
            the [x,y,z] offset inside the reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        J = None
        J_func = None
        filename = name + "[0,0,0]" if np.allclose(x, 0) else name
        filename += "_J"

        # check to see if should try to load functions from file
        J, J_func = self._load_from_file(filename, lambdify)

        if J is None and J_func is None:
            # if no saved file was loaded, generate function
            print("Generating Jacobian function for %s" % filename)

            Tx = self._calc_Tx(name, x=x, lambdify=False)
            # NOTE: calculating the Jacobian this way doesn't incur any
            # real computational cost (maybe 30ms) and it simplifies adding
            # the orientation information below (as opposed to using
            # sympy's Tx.jacobian method)
            # TODO: rework to use the Jacobian function and automate
            # derivation of the orientation Jacobian component
            J = []
            # calculate derivative of (x,y,z) wrt to each joint
            for ii in range(self.N_JOINTS):
                J.append([])
                J[ii].append(Tx[0].diff(self.q[ii]))  # dx/dq[ii]
                J[ii].append(Tx[1].diff(self.q[ii]))  # dy/dq[ii]
                J[ii].append(Tx[2].diff(self.q[ii]))  # dz/dq[ii]

            if "EE" in name:
                end_point = self.N_JOINTS
            elif "link" in name:
                end_point = int(name.strip("link"))
            elif "joint" in name:
                end_point = int(name.strip("joint"))
            # can't have more joint derivatives than there are joints
            end_point = min(end_point, self.N_JOINTS)

            # add on the orientation information up to the last joint
            for ii in range(end_point):
                J[ii] = J[ii] + list(self.J_orientation[ii])
            # fill in the rest of the joints orientation info with 0
            for ii in range(end_point, self.N_JOINTS):
                J[ii] = J[ii] + [0, 0, 0]
            J = sp.Matrix(J).T  # correct the orientation of J

            # save to file
            abr_control.utils.os_utils.makedirs(
                "%s/%s" % (self.config_folder, filename)
            )
            cloudpickle.dump(
                J, open("%s/%s/%s" % (self.config_folder, filename, filename), "wb")
            )

        if lambdify is False:
            # if should return expression not function
            return J

        if J_func is None:
            J_func = self._generate_and_save_function(
                filename=filename, expression=J, parameters=self.q + self.x
            )
        return J_func

    def _calc_M(self, lambdify=True):
        """ Uses Sympy to generate the inertia matrix in joint space

        Parameters
        ----------
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        M = None
        M_func = None

        # check to see if we have our inertia matrix saved in file
        M, M_func = self._load_from_file("M", lambdify)

        if M is None and M_func is None:
            # if no saved file was loaded, generate function
            print("Generating inertia matrix function")

            # get the Jacobians for each link's COM
            J_links = [
                self._calc_J("link%s" % ii, x=self.x_zeros, lambdify=False)
                for ii in range(self.N_LINKS)
            ]
            J_joints = [
                self._calc_J("joint%s" % ii, x=self.x_zeros, lambdify=False)
                for ii in range(self.N_JOINTS)
            ]

            # sum together the effects of each arm segment's inertia
            M = sp.zeros(self.N_JOINTS)
            for ii in range(self.N_LINKS):
                # transform each inertia matrix into joint space
                M += J_links[ii].T * self._M_LINKS[ii] * J_links[ii]
            # sum together the effects of each joint's inertia on each motor
            for ii in range(self.N_JOINTS):
                # transform each inertia matrix into joint space
                M += J_joints[ii].T * self._M_JOINTS[ii] * J_joints[ii]
            M = sp.Matrix(M)

            # save to file
            abr_control.utils.os_utils.makedirs("%s/M" % (self.config_folder))
            cloudpickle.dump(M, open("%s/M/M" % self.config_folder, "wb"))

        if lambdify is False:
            # if should return expression not function
            return M

        if M_func is None:
            M_func = self._generate_and_save_function(
                filename="M", expression=M, parameters=self.q
            )
        return M_func

    def _calc_R(self, name, lambdify=True):
        """ Uses Sympy to generate the rotation matrix for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """
        R = None
        R_func = None
        filename = name + "_R"

        # check to see if we have the rotation matrix saved in file
        R, R_func = self._load_from_file(filename, lambdify=True)

        if R is None and R_func is None:
            # if no saved file was loaded, generate function
            print("Generating rotation matrix function.")
            R = self._calc_T(name=name)[:3, :3]

            # save to file
            abr_control.utils.os_utils.makedirs(
                "%s/%s" % (self.config_folder, filename)
            )
            cloudpickle.dump(
                sp.Matrix(R),
                open("%s/%s/%s" % (self.config_folder, filename, filename), "wb"),
            )

        if R_func is None:
            R_func = self._generate_and_save_function(
                filename=filename, expression=R, parameters=self.q
            )
        return R_func

    def _calc_C(self, lambdify=True):
        """ Uses Sympy to generate the centrifugal and Coriolis forces
        Derivation from Robot Dynamics and Control by (Spong, Hutchinson,
        and Vidyasagar, 2004)

        Parameters
        ----------
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        C = None
        C_func = None
        # check to see if we have our term saved in file
        C, C_func = self._load_from_file("C", lambdify)

        if C is None and C_func is None:
            # if no saved file was loaded, generate function
            print("Generating centrifugal and Coriolis compensation function")

            # first get the inertia matrix
            M = self._calc_M(lambdify=False)

            # C_{kj} = sum_i c_{ijk}(q) \dot{q}_i
            # c_{ijk} = 1/2 * sum_i (\frac{\partial M_{kj}}{\partial q_j} +
            # \frac{\partial M_{ki}}{\partial q_j} - \frac{\partial M_{ij}}
            # {\partial q_k})
            C = sp.zeros(self.N_JOINTS, self.N_JOINTS)
            for kk in range(self.N_JOINTS):
                for jj in range(self.N_JOINTS):
                    for ii in range(self.N_JOINTS):
                        dMkjdqi = M[kk, jj].diff(self.q[ii])
                        dMkidqj = M[kk, ii].diff(self.q[jj])
                        dMijdqk = M[ii, jj].diff(self.q[kk])
                        C[kk, jj] += 0.5 * (dMkjdqi + dMkidqj - dMijdqk) * self.dq[ii]
                    C[kk, jj] = C[kk, jj]
            C = sp.Matrix(C)

            # save to file
            abr_control.utils.os_utils.makedirs("%s/C" % self.config_folder)
            cloudpickle.dump(C, open("%s/C/C" % self.config_folder, "wb"))

        if lambdify is False:
            # if should return expression not function
            return C

        if C_func is None:
            C_func = self._generate_and_save_function(
                filename="C", expression=C, parameters=self.q + self.dq
            )
        return C_func

    def _calc_T(self, name):
        """ Uses Sympy to generate the transform for a joint or link

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        """
        raise NotImplementedError("_calc_T function not implemented")

    def _calc_Tx(self, name, x=None, lambdify=True):
        """Return transform from x in reference frame of 'name' to the origin

        Uses Sympy to transform x from the reference frame of a joint
        or link to the origin (world) coordinates.

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        x : numpy.array
            the [x,y,z] offset inside the reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        Tx = None
        Tx_func = None
        filename = name + "[0,0,0]" if np.allclose(x, 0) else name
        filename += "_Tx"
        # check to see if we have our transformation saved in file
        Tx, Tx_func = self._load_from_file(filename, lambdify)

        if Tx is None and Tx_func is None:
            print("Generating transform function for %s" % filename)
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
            abr_control.utils.os_utils.makedirs(
                "%s/%s" % (self.config_folder, filename)
            )
            cloudpickle.dump(
                sp.Matrix(Tx),
                open("%s/%s/%s" % (self.config_folder, filename, filename), "wb"),
            )

        if lambdify is False:
            # if should return expression not function
            return Tx

        if Tx_func is None:
            Tx_func = self._generate_and_save_function(
                filename=filename, expression=Tx, parameters=self.q + self.x
            )
        return Tx_func

    def _calc_T_inv(self, name, x, lambdify=True):
        """ Return the inverse transform matrix

        Return the inverse transform matrix, which converts from
        world coordinates into the robot's end-effector reference frame

        Parameters
        ----------
        name : string
            name of the joint, link, or end-effector
        x : numpy.array
            the [x,y,z] offset inside the reference frame of 'name' [meters]
            if not specified, (0, 0, 0) is hard coded in, rather than using
            variable (x, y, z), which results in significant speedups.
        lambdify : boolean, optional (Default: True)
            if True returns a function to calculate the matrix.
            If False returns the Sympy matrix
        """

        T_inv = None
        T_inv_func = None
        filename = name + "[0,0,0]" if np.allclose(x, 0) else name
        filename += "_Tinv"
        # check to see if we have our transformation saved in file
        T_inv, T_inv_func = self._load_from_file(filename, lambdify)

        if T_inv is None and T_inv_func is None:
            print("Generating inverse transform function for %s" % filename)
            T = self._calc_T(name=name)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]])
            )
            T_inv = sp.Matrix(T_inv)

            # save to file
            abr_control.utils.os_utils.makedirs(
                "%s/%s" % (self.config_folder, filename)
            )
            cloudpickle.dump(
                T_inv, open("%s/%s/%s" % (self.config_folder, filename, filename), "wb")
            )

        if lambdify is False:
            # if should return expression not function
            return T_inv

        if T_inv_func is None:
            T_inv_func = self._generate_and_save_function(
                filename=filename, expression=T_inv, parameters=self.q + self.x
            )
        return T_inv_func
