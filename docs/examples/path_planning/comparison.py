from abr_control.controllers import path_planners as pp
import numpy as np
import matplotlib.pyplot as plt

pos = np.random.uniform(low=-3, high=3, size=3)
ori = np.random.uniform(low=-3.14, high=3.14, size=3)

t_pos = np.random.uniform(low=-10, high=10, size=3)
t_ori = np.random.uniform(low=-3.14, high=3.14, size=3)

# NOTE not including inverse_kinematics as this is a joint space planner

#=====LINEAR======
# want to reach target with a constant step size
# can be defined with dx or n_timesteps
# return position and velocity
print('LINEAR VELOCITY')
lin = pp.Linear(
        n_timesteps=1000,
        dt=0.001)
p, v = lin.generate_path(
        position=pos,
        target_position=t_pos,
        plot=True)

#====GAUSSIAN====
# want to move to a point in a straight line, with a smooth
# gaussian velocity profile, limited by max_v and max_a
print('GAUSSIAN VELOCITY')
gaus = pp.GaussianPathPlanner(
        max_a=5,
        max_v=5,
        dt=0.001
)
# returns pos, vel, ori
pvo = gaus.generate_path(
        lin_state=np.hstack((pos, np.zeros(3))),
        ang_state=ori,
        lin_target=np.hstack((t_pos, np.zeros(3))),
        ang_target=t_ori,
        start_v=0,
        end_v=3,
        plot=True
    )

#====ARC====
# creates an arc path between start and target, where the arc
# is a linear combination of the arc sections between start
# and target relative to the origin, of the circles with radius
# equivalent to the distance from start and target to origin,
# respectively
# tldr; run to see plot for better visual explanation
print('ARC POSITION')
arc = pp.Arc(n_timesteps=1000)
p, v = arc.generate_path(
        position=pos,
        target_position=t_pos,
        plot=True)

#====DMP=====
# second order dmp path
print('2nd ORDER DMP')
dmp = pp.SecondOrderDMP(n_timesteps=1000)
p, v = dmp.generate_path(
        position=pos,
        target_position=t_pos,
        plot=True)

#====2nd Order Filter====
print('2nd ORDER FILTER')
filt = pp.SecondOrderFilter(n_timesteps=1000)
p, v = filt.generate_path(
        position=pos,
        target_position=t_pos,
        velocity=np.array([1, 0, 0]),
        plot=True)
