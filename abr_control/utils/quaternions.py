""" Set of helper functions to do math and conversions with quaternions.
"""
import numpy as np


def quat_mult(q0, q1):
    """ Multiply two quaternions.

    q0 np.array: the first quaternion [w, x, y, z]
    q1 np.array: the second quaternion [w, x, y, z]
    """
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1
    z = w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1
    return np.array([w, x, y, z])

def quat_conj(q):
    """ Calculate the conjugate of a quaternion.

    q np.array: the quaternion [w, x, y, z]
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def rot2quat(R):
    """ Convert the rotation matrix of R into a quaternion, equations from
    euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion

    R np.array: the rotation matrix to convert into a quaternion
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return np.array([qw, qx, qy, qz])

def eul2quat(angles, order='xyz'):
    """ Converts from Euler angles into a quaternion.

    angles np.array: the Euler angle (radians) [alpha, beta, gamma]
    order string: specifies the order of application of the rotation
                        matrices Rx, Ry, and Rz. Default xyz matches VREP.
    """
    alpha, beta, gamma = angles

    qx = [np.cos(alpha / 2.0), np.sin(alpha / 2.0), 0, 0];
    qy = [np.cos(beta / 2.0), 0, np.sin(beta / 2.0), 0];
    qz = [np.cos(gamma / 2.0), 0, 0, np.sin(gamma / 2.0)];

    if order == 'xyz':
        q = quat_mult(qx, quat_mult(qy, qz))
    elif order == 'xzy':
        q = quat_mult(qx, quat_mult(qz, qy))
    elif order == 'yxz':
        q = quat_mult(qy, quat_mult(qx, qz))
    elif order == 'yzx':
        q = quat_mult(qy, quat_mult(qz, qx))
    elif order == 'zxy':
        q = quat_mult(qz, quat_mult(qx, qy))
    elif order == 'zyx':
        q = quat_mult(qz, quat_mult(qy, qx))
    else:
        raise Exception('Euler angle order not supported.')

    return q

def quat2eul(q, order='xyz'):
    """ Convert a quaternion to Euler angles.

    q np.array: the quaternion [w, x, y, z]
    order string: specifies the order of application of the rotation
                        matrices Rx, Ry, and Rz. Default xyz matches VREP.
    """
    w, x, y, z = q

    def twoaxisrot(r11, r12, r21, r31, r32):
        alpha = np.arctan2(r11, r12)
        beta = np.arccos(r21)
        gamma = np.arctan2(r31, r32)
        return np.array([alpha, beta, gamma])

    def threeaxisrot(r11, r12, r21, r31, r32):
        alpha = np.arctan2(r31, r32)
        beta = np.arcsin (r21)
        gamma  = np.arctan2(r11, r12)
        return np.array([alpha, beta, gamma])

    if order == 'zyx':
        angles = threeaxisrot(
            2 * (x * y + w * z),
            w * w + x * x - y * y - z * z,
            -2 * (x * z - w * y),
            2 * (y * z + w * x),
            w * w - x * x - y * y + z * z)

    elif order == 'zyz':
        angles = twoaxisrot(
            2 * (y * z - w * x),
            2 * (x * z + w * y),
            w * w - x * x - y * y + z * z,
            2 * (y * z + w * x),
            -2 * (x * z - w * y))

    elif order == 'zxy':
        angles = threeaxisrot(
            -2 * (x * y - w * z),
            w * w - x * x + y * y - z * z,
            2 * (y * z + w * x),
            -2 * (x * z - w * y),
            w * w - x * x - y * y + z * z)

    elif order == 'zxz':
        angles = twoaxisrot(
            2 * (x * z + w * y),
            -2 * (y * z - w * x),
            w * w - x * x - y * y + z * z,
            2 * (x * z - w * y),
            2 * (y * z + w * x))

    elif order == 'yxz':
        angles = threeaxisrot( 2 * (x * z + w * y),
            w * w - x  * x - y * y + z * z,
            -2 * (y * z - w * x),
            2 * (x * y + w * z),
            w * w - x  * x + y * y - z * z)

    elif order == 'yxy':
        angles = twoaxisrot(
            2 * (x * y - w * z),
            2 * (y * z + w * x),
            w * w - x * x + y * y - z * z,
            2 * (x * y + w * z),
            -2 * (y * z - w * x))

    elif order == 'yzx':
        angles = threeaxisrot(
            -2 * (x * z - w * y),
            w * w + x * x - y * y - z * z,
            2 * (x * y + w * z),
            -2 * (y * z - w * x),
            w * w - x * x + y * y - z * z,)

    elif order == 'yzy':
        angles = twoaxisrot(
            2 * (y * z + w * x),
            -2 * (x * y - w * z),
            w * w - x * x + y * y - z * z,
            2 * (y * z - w * x),
            2 * (x * y + w * z))

    elif order == 'xyz':
        angles = threeaxisrot(
            -2  * (y* z - w * x),
            w * w - x * x - y * y + z  * z,
            2 * (x * z + w * y),
            -2  * (x * y - w * z),
            w * w + x * x - y * y - z * z,)

    elif order == 'xyx':
        angles = twoaxisrot(
            2 * (x * y + w * z),
            -2 * (x * z - w * y),
            w * w + x * x - y * y - z * z,
            2 * (x * y - w * z),
            2 * (x * z + w * y))

    elif order == 'xzy':
        angles = threeaxisrot(
            2 * (y * z + w * x),
            w * w - x * x + y * y - z * z,
            -2 * (x * y - w * z),
            2 * (x * z + w * y),
            w * w + x * x - y * y - z * z)

    elif order == 'xzx':
        angles = twoaxisrot(
            2 * (x * z - w * y),
            2 * (x * y + w * z),
            w * w + x * x - y * y - z * z,
            2 * (x * z + w * y),
            -2 * (x * y - w * z),)

    else:
        raise Exception('Unknown rotation sequence')

    return angles


if __name__ == '__main__':

    # angles = np.array([-163, 90, -131]) * np.pi / 180.0
    angles = [1, 2, -1]
    order = 'zyx'
    q = eul2quat(angles, order=order)
    angles2 = quat2eul(q, order=order)
    print('angles: ', angles)
    print('quaternion: ', q)
    print('angles2: ', angles2)
