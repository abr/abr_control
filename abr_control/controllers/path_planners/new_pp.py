import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate

# define parameters
dt = 0.001
max_v = 2
max_a = 1
# curve_weight = 5
start_v = 0
end_v = 0
np.random.seed(10)

def yaw_pitch_alignment(a, b):
    def get_angle(a, b):
        theta = np.arccos(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)))
        return theta

    alpha = get_angle(a[:2], b[:2])
    Ryaw = np.array([
        [np.cos(alpha), -np.sin(alpha), 0],
        [np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 1]]
    )
    beta = get_angle(a, b)
    Rpitch = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]]
    )
def align_vectors(a, b):
    """
    Takes vectors a and b and returns rotation matrix to align a to b
    """
    b = b / np.linalg.norm(b) # normalize a
    a = a / np.linalg.norm(a) # normalize b
    v = np.cross(a, b)
    c = np.dot(a, b)

    v1, v2, v3 = v
    h = 1 / (1 + c)

    Vmat = np.array([[0, -v3, v2],
                  [v3, 0, -v1],
                  [-v2, v1, 0]])

    R = np.eye(3, dtype=np.float64) + Vmat + (Vmat.dot(Vmat) * h)
    return R


def get_gauss_curve(vel, max_v, max_a, dt):
    """
    generates the left half of the gaussian curve with 3std, with sigma determined
    by the timestep and max_a
    """
    assert vel <= max_v

    if vel == max_v:
        vel_profile = None
    else:
        # calculate the time needed to reach our maximum velocity from vel
        ramp_up_time = (max_v-vel)/max_a

        # Amplitude of Gaussian is max speed, get our sigma from here
        s = 1/ ((max_v-vel) * np.sqrt(np.pi*2))

        # Go 3 standard deviations so our tail ends get close to zero
        u = 3*s

        # We are generating the left half of the gaussian, so generate our
        # x values from 0 to the mean, with the steps determined by our
        # ramp up time (which itself is determined by max_a)
        x = np.linspace(0, u, int(ramp_up_time/dt))

        # Get our gaussian y values, which is our normalized velocity profile
        vel_profile = 1 * (
                1/(s*np.sqrt(2*np.pi)) * np.exp(-0.5*((x-u)/s)**2)
        )

        # Since the gaussian goes off to +/- infinity, we need to shift our
        # curve down so that it starts at zero
        vel_profile -= vel_profile[0]

        # scale back up so we reach our target v at the end of the curve
        vel_profile *= ((max_v-vel) / vel_profile[-1])

        # add to our baseline starting velocity
        vel_profile += vel

    return vel_profile

# get velocity profiles
starting_vel_profile = get_gauss_curve(
        vel=start_v,
        max_v=max_v,
        max_a=max_a,
        dt=dt
)
starting_dist = np.sum(starting_vel_profile*dt)
print('ramp up distance: ', starting_dist)
ending_vel_profile = get_gauss_curve(
        vel=end_v,
        max_v=max_v,
        max_a=max_a,
        dt=dt
        )[::-1]
ending_dist = np.sum(ending_vel_profile*dt)
print('ramp down distance: ', ending_dist)


# Define our target path curve
def Plinear(t):
    """
    A function defining the shape of our path from start to target
    Restrictions
    ------------
    - the path must start at [0, 0, 0] and end at [1, 1, 1]
        these are the start and target position, respectively
        The reason for this is that we just need to define the shape of
        our path with respect to the straight line path from start to
        target. The path planner will do the stretching so that the path
        at t==0 will be the start, and will end at the target
    """
    x = t
    y = t
    z = t
    return np.array([x, y, z])

def Pcurve(t):
    z = np.sin(t*np.pi/2)
    # y = np.sin(5*t*np.pi/2)
    y = t
    x = t
    return np.array([x, y, z])


# P = Plinear
P = Pcurve
assert sum(P(0)) < 1e-9
assert sum(P(1))-3 < 1e-9

t = np.linspace(0, 1, 100)
# random start and target
start = np.random.uniform(low=-10, high=10, size=3)
target = np.random.uniform(low=-10, high=10, size=3)
target_vector = target - start
norm_vector = target_vector/np.linalg.norm(target_vector)
base = np.array([1, 1, 1])
base_norm = base/np.linalg.norm(base)
R = align_vectors(base_norm, norm_vector)


def get_length(func, steps=1000, **kwargs):
    """
    calculate the distance travelled in our curve
    in the domain of [0, 1]
    """
    y = []
    dist_steps = []
    for ii, t in enumerate(np.linspace(0, 1, steps)):
        y.append(func(t, **kwargs))
        if t>0:
            dist_steps.append(np.linalg.norm(y[ii]-y[ii-1]))
        else:
            dist_steps.append(0)
    print(f"Total distance covered using {steps} steps: {np.sum(dist_steps)}")
    return dist_steps


def length_func(func, steps=1000, **kwargs):
    #returns a function that returns the xyz positions that are distance d along func
    xyz = []
    dist_steps = []
    for ii, t in enumerate(np.linspace(0, 1, steps)):
        xyz.append(func(t, **kwargs))
        if t>0:
            dist_steps.append(np.linalg.norm(xyz[ii]-xyz[ii-1]))
        else:
            dist_steps.append(0)
    dist_steps = np.cumsum(dist_steps)
    xyz = np.array(xyz)
    X = scipy.interpolate.interp1d(dist_steps, xyz.T[0])
    Y = scipy.interpolate.interp1d(dist_steps, xyz.T[1])
    Z = scipy.interpolate.interp1d(dist_steps, xyz.T[2])
    return [X, Y, Z]

def shifted_func(t, profile, start, target, R, **kwargs):
    """
    Combines our profile and ramp
    """
    # y = (target-start) * np.dot(R, profile(t)) + start #ramp(start, target, t)
    scale = np.linalg.norm(target-start)
    rot = np.dot(R, (1/np.sqrt(3))*profile(t)*scale) + start
    return rot

curve_dist_steps = get_length(
        shifted_func,
        profile=P,
        # ramp=linear,
        start=start,
        target=target,
        R=R)
curve_length = np.sum(curve_dist_steps)
# get positions as a function of distance along the curve
XYZ = length_func(
        shifted_func,
        profile=P,
        # ramp=linear,
        start=start,
        target=target,
        R=R)


if curve_length >= starting_dist + ending_dist:
    print('ADDING LINEAR PORTION IN MIDDLE OF PATH')
    # calculate the remaining steps where we will be at constant max_v
    remaining_dist = curve_length - (ending_dist + starting_dist)
    constant_speed_steps = int(remaining_dist/ max_v / dt)

    vel_profile = np.hstack((
        starting_vel_profile,
        np.ones(constant_speed_steps) * max_v,
        ending_vel_profile
    ))
else:
    print('PATH TOO SHORT, COMPRESSING VELOCITY PROFILE')
    # scale our profile
    # TODO to do this properly we should evaluate the integral to get the t where
    # the sum of the profile is half our travel distance. This way we maintain
    # the same acceleration profile instead of maintaining the same number of steps
    # and accelerating more slowly
    # NOTE ERROR: if we have non-zero start or end velocities this scales us away from
    # velocity
    scale = curve_length / (starting_dist + ending_dist)
    vel_profile = np.hstack((
        scale*starting_vel_profile,
        scale*ending_vel_profile
    ))

plt.figure()
plt.title('Velocity Profile Used to Generate Path')
plt.plot(vel_profile)
# Generate our position path from the velocity curve
position_path = [start]
# ramp = []
curve = []
shifted = []

# plt.figure()
# vel_spacing = vel_profile*dt
# # plt.plot(vel_spacing, label='positions')
# vel_spacing = np.cumsum(vel_spacing)
# # print(f"{vel_spacing[-1]=}")
# # print(f"{starting_dist=}")
# # plt.plot(vel_spacing, label='cumsum')
# vel_spacing = np.diff(vel_spacing)
# # plt.plot(vel_spacing, label='diff')
# plt.scatter(vel_spacing, vel_spacing, label='diff', s=1)
# # vel_spacing /= curve_length
# # plt.plot(vel_spacing, label='norm spacing')
# # plt.legend()
# plt.show()
path_steps = np.cumsum(vel_profile*dt)
for ii in range(1, len(vel_profile)):
# for ii, tt in enumerate(vel_spacing):
    # normalize step for the normalized shape functions
    tt = path_steps[ii]/curve_length#max(abs(vel_profile)

    # r1 = linear(start, target, tt)
    c1 = P(tt)#*curve_weight
    # shift1 = shifted_func(t=tt, profile=P, start=start, target=target)#r1+c1
    shiftx = XYZ[0](path_steps[ii])
    shifty = XYZ[1](path_steps[ii])
    shiftz = XYZ[2](path_steps[ii])
    shift1 = np.array([shiftx, shifty, shiftz])

    # r2 = linear(start, target, tt2)
    # c2 = P(tt2)*curve_weight
    # shift2 = r2+c2
    direction = shift1-position_path[-1]
    norm_dir = direction/np.linalg.norm(direction)

    # ramp.append(r1)
    curve.append(c1)
    shifted.append(shift1)
    position_path.append(shift1)

    # position_path.append(
    #         position_path[-1] + vel_profile[ii]*dt*(norm_dir)
    # )

shifted = np.asarray(shifted).T
print('SHIFTED: ', shifted.shape)
# ramp = np.asarray(ramp).T
curve = np.asarray(curve).T
# print(f"{curve.shape=}")

position_path = np.asarray(position_path)
# print((position_path[2]-position_path[1])/dt)
# print(position_path[2])
# print(position_path[1])

# get our 3D vel profile components by differentiating the position path
velocity_path = np.asarray(np.gradient(position_path, dt, axis=0))


# ===== plotting
# plot the input curve that defines the path shape
plt.figure()
ax1 = plt.subplot(121, projection='3d')
ax1.set_title('Normalized Curve')
ax1.plot(curve[0], curve[1], curve[2])
ax1.set_xlim3d(-5, 5)
ax1.set_ylim3d(-5, 5)
ax1.set_zlim3d(-5, 5)

# plot the linear ramp from start to target
# ax2 = plt.subplot(132, projection='3d')
# ax2.set_title('Linear Ramp')
# ax2.plot(ramp[0], ramp[1], ramp[2])
# ax2.scatter(start[0], start[1], start[2], label='start')
# ax2.scatter(target[0], target[1], target[2], label='target')
# ax2.legend()

# plot the combined curve that follows the curve shape, but
# goes from start to target
ax3 = plt.subplot(122, projection='3d')
ax3.set_xlim3d(-5, 5)
ax3.set_ylim3d(-5, 5)
ax3.set_zlim3d(-5, 5)
ax3.set_title('Shifted Curve')
ax3.plot(shifted[0], shifted[1], shifted[2])
ax3.scatter(start[0], start[1], start[2], label='start')
ax3.scatter(target[0], target[1], target[2], label='target')
ax3.legend()

# plot the components of
# - the desired curve
# - the shifted curve that goes from start to final
# - the final path sampled at values that follow the velocity profile
plt.figure()
plt.subplot(3,3,1)
plt.title('Noralized X Shape')
plt.plot(curve[0])
plt.subplot(3,3,2)
plt.title('Noralized Y Shape')
plt.plot(curve[1])
plt.subplot(3,3,3)
plt.title('Noralized Z Shape')
plt.plot(curve[2])

plt.subplot(3,3,4)
plt.title('Warped X Path')
plt.plot(shifted[0])
plt.subplot(3,3,5)
plt.title('Warped Y Path')
plt.plot(shifted[1])
plt.subplot(3,3,6)
plt.title('Warped Z Path')
plt.plot(shifted[2])

plt.subplot(3,3,7)
plt.title('X Path')
plt.plot(position_path[:, 0])
plt.subplot(3,3,8)
plt.title('Y Path')
plt.plot(position_path[:, 1])
plt.subplot(3,3,9)
plt.title('Z Path')
plt.plot(position_path[:, 2])
plt.tight_layout()

# plot the velocity profile
plt.figure()
plt.title('Velocity')
plt.plot(velocity_path[:, 0], 'r')
plt.plot(velocity_path[:, 1], 'b')
plt.plot(velocity_path[:, 2], 'g')

norm = []
for vel in velocity_path:
    norm.append(np.linalg.norm(vel))
plt.plot(norm, 'y')
plt.plot([max_v]*len(norm), linestyle='--')
plt.plot([start_v]*len(norm), linestyle='--')
plt.plot([end_v]*len(norm), linestyle='--')
plt.legend(['dx', 'dy', 'dz', 'norm', 'vel limit', 'start_v', 'end_v'])


plt.show()


