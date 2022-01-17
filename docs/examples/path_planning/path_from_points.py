from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import FromPoints
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
import numpy as np

pts = np.array([
    [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
    [0, 0.1, 0.15, 0.2, 0.25, 0.5, 0.75, 0.8, 0.85, 0.9, 1.0],
    [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]]
)
t = np.linspace(0, 1, pts.shape[1])
Pprof = FromPoints(pts=pts, x=t)
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
