.. image:: https://i.imgur.com/Tu5GnX3.jpg

***********
ABR Control
***********

The ABR Control library is a python package for the control and path planning of
robotic arms in real or simulated environments. ABR Control provides API's for the
Mujoco, CoppeliaSim (formerly known as VREP), and Pygame simulation environments, and
arm configuration files for one, two, and three-joint models, as well as the UR5 and
Kinova Jaco 2 arms. Users can also easily extend the package to run with custom arm
configurations. ABR Control auto-generates efficient C code for generating the control
signals, or uses Mujoco's internal functions to carry out the calculations.

ABR also provides an interface and config available for controlling a real Jaco 2
at the `ABR_Jaco2 <https://github.com/abr/abr_jaco2/>`_ repository.

Installation
============

The ABR Control library depends on NumPy, SymPy, SciPy, CloudPickle, Cython,
SetupTools, Nengo, and Matplotlib. We recommend using
`Anaconda <https://store.continuum.io/cshop/anaconda/>`_.
Note that installing in a clean environment will require compiling of the dependent
libraries, and will take a few minutes.

To install ABR Control, clone this repository and run::

    sudo apt-get install g++
    sudo apt-get install python-dev
    sudo apt-get install libfreetype6-dev
    conda activate your_environment
    python setup.py install
    python setup.py develop

ABR Control is tested to work on Python 3.6+, Python 2 is not supported.

Optional Installation
=====================

Mujoco
------
If you would like to use the Mujoco API you will need to install a
forked version of `mujoco-py <https://github.com/studywolf/mujoco-py/>`_ with hooks for
exitting out of simulations with the ESC key. To use the mujoco API, make sure you are
in your anaconda environment and run::

    git clone https://github.com/studywolf/mujoco-py.git
    cd mujoco-py
    pip install -e .
    pip install glfw=>1.8.3
    pip install requests

Pygame
------
If you would like to use the Pygame API, from your anaconda environment run::

    pip install pygame

Vrep
----
We support Vrep <=4.0. You will need to download
`Vrep <http://coppeliarobotics.com/previousVersions/>`_ and follow the installation
instructions.

PyDMPs
------
Some of the path planners work through the use of dynamic movement primitives (DMPs).
DMPs allow for a stable, generalizable, and easily extensible  way of representing
complex trajectories. Path planners making use of DMP are appended with 'dmp' in their
filename and will require the installation of the pydmps repository. To install, from
your Anaconda environment run::

    pip install pydmps


Usage
=====

The ABR Control repo is comprised of three parts: 1) arms, 2) controllers, and
3) interfaces.

1a) Arms: Using CoppeliaSim, Pygame, or a real arm
---------------------------------------
All of the required information about an arm model is kept in that arm's config file.
To use the ABR Control library with a new arm, the user must provide the transformation
matrices (written using SymPy expressions) from the robot's origin reference frame to
each link's centre-of-mass (COM) and joints. These are specified sequentially, e.g.
origin -> link0 COM, link0 COM -> joint0, joint0 -> link1 COM, etc. The arm config file
and any simulation code is kept in a folder named the same as the arm in the
``abr_control/arms/`` directory.

The ABR Control configuration base class uses the SymPy transform matrices to provide
functions that will calculate the transforms, Jacobian, Jacobian derivative, inertia
matrices, gravity forces, and centripetal and Coriolis effects for each joint and COM.
These can be accessed::

    from abr_control.arms import jaco2

    robot_config = jaco2.Config()
    # calculate the following given the arm state at joint_angles
    robot_config.Tx('joint3', q=joint_angles)  # the (x, y, z) position of joint3
    robot_config.M(q=joint_angles)  # calculate the inertia matrix in joint space
    robot_config.J('EE', q=joint_angles)  # the Jacobian of the end-effector

By default, the ``use_cython`` parameter is set to ``True`` to allow for real-time
control by generating optimized Cython code for each of the robot configuration
functions. This can take a little bit of time to generate these functions, but they
are saved in `~.cache/abr_control/arm_name/saved_functions` where they will be loaded
from for future runs. Note that a hash is saved for the config, so if any changes are
made the functions will be regenerated during the next use. The cython optimization can
be turned off on instantiation::

    from abr_control.arms import ur5

    robot_config = ur5.Config(use_cython=False)

Below are results from running the operational space controller with different
controllers with ``use_cython=True`` and ``False``.

.. image:: examples/timing.png

1b) Arms: Using Mujoco
----------------------
When using Mujoco the process is a bit different. Mujoco handles the calculation of all
the kinematics and dynamics functions, and only requires an xml config be made
describing the kinematic chain. The
`Mujoco API <http://www.mujoco.org/book/modeling.html>`_ page describes this in detail.

Detailed models can be created by importing 3D modeling stl files and using the
``mesh`` object type in the ``<geom>`` tag. An example of this is the
``abr_control/arms/jaco2/jaco2.xml``.  For users building their own models, you may
specify the location of the xml with the ``folder`` parameter. For more details, please
refer to the Mujoco documentation linked above and use the xml files in this repository
as examples.

2) Controllers
--------------
Controllers make use of the robot configuration files to generate control signals that
accomplish a given task (for most controllers this is reaching a target). The ABR
Control library provides implementations of several primary controllers, including
operational space, generalized coordinates (joint) space, sliding, and floating
control.

When using an operational space controller, it is possible to also pass in secondary
controllers to operate in the null space of the operational space controller. These
secondary controllers can be set up to achieve secondary goals such as avoiding joint
limits and obstacles, damping movement, or maintaining a configuration near a specified
resting state.

In the ``path_planners`` folder there are several path planners that can be used in
conjunction with the controllers. There are filters, linear and second order, which can
be used to trace a path from the current position to the target without suddenly
warping and causing large spikes in generated torque. The inverse kinematics planner
takes in a target for the end-effector and returns a joint angle trajectory to follow.
An arc path planner is also provided that creates an arcing path which can be useful
when the arm has to reach over itself. This can help prevent self-collisions and odd
arm configurations.

Each path planner also has the ability to generate a trajectory for end-effector
orientation with the ``path_plannner.generate_orientation_path()`` function. This uses
spherical linear interpolation (SLERP) to generate a set of orientations from a start
to a target quaternion. The time profile will match that of the path planner
instantiated (ie: a linear path planner will have a linear step in orientation over
time, with a constant change in orientation, whereas a second order path planner will
have a bell shaped profile with the largest steps occurring during the middle of the
movement, with an acceleration and deceleration at the start and end, respectively.)
In addition to filters, there is an example path planner using the dynamic movement
primitives trajectory generation system.

Finally, there is an implementation of nonlinear adaptive control in the ``signals``
folder, as well as examples in Mujoco, PyGame, and CoppeliaSim showing how this class
can be used to overcome unexpected forces acting on the arm.

3) Interfaces
-------------
For communications to and from the system under control, an interface API is used.
The functions available in each class vary depending on the specific system, but must
provide ``connect``, ``disconnect``, ``send_forces`` and ``get_feedback`` methods.

Putting everything together
---------------------------
A control loop using these three files looks like::

    from abr_control.arms import jaco2
    from abr_control.controllers import OSC
    from abr_control.interfaces import CoppeliaSim

    robot_config = jaco2.Config()
    interface = CoppeliaSim(robot_config)
    interface.connect()

    ctrlr = OSC(robot_config, kp=20,
                # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
                ctrlr_dof=[True, True, True, False, False, False])

    target_xyz = [.2, .2, .5]  # in metres
    target_orientation = [0, 0, 0]  # Euler angles, relevant when controlled
    for ii in range(1000):
        feedback = interface.get_feedback()  # returns a dictionary with q, dq
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack([target_xyz, target_orientation]))
        interface.send_forces(u)  # send forces and step CoppeliaSim sim forward

    interface.disconnect()

**NOTE** that when using the Mujoco interface it is necessary to instantiate and
connect the interface before instantiating the controller. Some parameters only get
parsed from the xml once the arm config is linked to the mujoco interface, which
happens upon connection.


Examples
========

The ABR Control repo comes with several examples that demonstrate the use of the
different interfaces and controllers.

By default all of the PyGame examples run with the three-link MapleSim arm. You can
also run the examples using the two-link Python arm by changing the import statement at
the top of the example scripts.

To run the CoppeliaSim examples, have the most recent CoppeliaSim version open. By
default, the CoppeliaSim examples all run with the UR5 or Jaco2 arm model. To change
this, change which arm folder is imported at the top of the example script. The first
time you run an example you will be promted to download the arm model. Simply select
``yes`` to download the file and the simulation will start once the download completes.

To run the Mujoco examples, you will be promted to download any mesh or texture files,
if they are used in the xml config, similarly to the CoppeliaSim arm model. Once the
download completes the simulation will start. If you are using the forked Mujoco-Py
repository (See Optional Installation section) you can exit the simulation with the ESC
key and pause with the spacebar.
