***********
ABR Control
***********

ABR_Control: Robotic arm control in Python

Installation
============

The ABR_Control library depends on NumPy, SymPy, SciPy, CloudPickle, and
Cython, and we recommend that you install these libraries before
ABR_Control. If you're not sure how to do this, we recommend using
`Anaconda <https://store.continuum.io/cshop/anaconda/>`_.
Note that installing in a clean environment will require compiling of the
dependent libraries, and will take a few minutes.

To install ABR_Control::

    pip install abr_control

If you'd like to install ABR_Control from source,
clone this repository and run::

    python setupy.py install
    python setup.py develop

ABR_Control is tested to work on Python 3.4+.

Usage
=====

The ABR_Control repo is comprised of three parts: 1) arms, 2) controllers, and
3) interfaces.

1) All of the required information about an arm model is kept in that arm's
config file. To use the ÁBR_Control library with a new arm, the user must
provide the transformation matrices (written using SymPy expressions ) from
the robot's origin reference frame to each link's centre-of-mass (COM) and
joints. These are specified sequentially, e.g.  origin -> link0 COM,
link0 COM -> joint0, joint0 -> link1 COM, etc. Additionally, the arm models
or simulation code is kept in the arm's folder.

The ABR_Control configuration base class uses the SymPy transform matrices
to provide functions that will calculate the transforms, Jacobian, Jacobian
derivative, inertia matrices, gravity forces, and centripetal and Coriolis
effects for each joint and COM. These can be accessed::

    from abr_control.arms import jaco2

    robot_config = jaco2.Config()
    robot_config.Tx('joint3', joint_angles)  # the (x, y, z) position of joint3
    robot_config.M(joint_angles)  # calculate the inertia matrix in joint space
    robot_config.J('EE', joint_angles)  # the Jacobian of the end-effector

2) The controllers make use of the robot configuration files to generate
control signals that drive the robot to a target. The ABR_Control library
provides implementations of operational space control, joint space control,
and a floating controller.

Additionally, there are signals and path planners that can be used in
conjunction with the controllers. See the `obstacle_avoidance` or
`linear_path_planning` files for examples on how to use these.

3) For communications to and from the system under control, an interface class
is used. The functions available in each class vary depending on the specific
system, but must provide `connect`, `disconnect`, `send_forces` and
`get_feedback` methods. A control loop using these three files looks like::

    from abr_control.arms import jaco2
    from abr_control.controllers import OSC
    from abr_control.interfaces import VREP

    robot_config = jaco2.Config()
    ctrlr = OSC(robot_config)
    interface = VREP(robot_config)

    interface.connect()

    target_xyz = [.2, .2 .5]  # in metres
    for ii in range(1000)
        feedback = interface.get_feedback()  # returns a dictionary with q, dq
        u = ctrlr.generate(feedback['q'], feedback['dq'], target_xyz)
        interface.send_forces(u)  # send forces and step VREP sim forward

    interface.disconnect()

Examples
========

The ABR_Control repo comes with several examples that demonstrate the use of
the different interfaces and controllers.

By default all of the PyGame examples run with the three-link MapleSim arm.
You can also run the examples using the two-link Python arm by changing the
import statement at the top of the example scripts. Note that to run the PyGame
examples, you will also need to install Pygame::

    pip install pygame

To run the VREP examples, have VREP version > 3.2 open, and load the .ttt
file from the corresponding `abr_control/arms/` folder for the arm of interest.
By default, the VREP examples all run with the UR5 arm model. To change this,
change which arm folder is imported at the top of the example script.
