from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.position_profiles import SinCurve
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
import numpy as np
dt = 0.001
max_a = 2
# Pprof = Linear()
Pprof = SinCurve(axes=['x', 'z'])
Vprof = Gaussian(dt=dt, a=max_a)

path = PathPlanner(
        pos_profile=Pprof,
        vel_profile=Vprof,
        max_v=2,
        dt=dt,
        n_sample_points=1000,
        verbose=True
    )
path.generate_path(
        position=np.zeros(3),
        target_position=np.array([5, 3, -2]),
        orientation=np.array([0, 0, 0]),
        target_orientation=np.array([0, 0, 3.14]),
        start_v=0,
        end_v=0,
        plot=True
    )
