from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.position_profiles import SinCurve
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
import numpy as np
# Pprof = Linear()
Pprof = SinCurve(axes=['x', 'z'], n_sample_points=1000)
Vprof = Gaussian(dt=0.001, acceleration=1)

path = PathPlanner(
        pos_profile=Pprof,
        vel_profile=Vprof,
        verbose=True
    )
path.generate_path(
        start_position=np.zeros(3),
        target_position=np.array([5, 3, -2]),
        start_orientation=np.array([0, 0, 0]),
        target_orientation=np.array([0, 0, 3.14]),
        max_velocity=2,
        start_velocity=0,
        target_velocity=0,
        plot=True
    )
