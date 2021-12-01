import numpy as np


def rotation_vector_to_quaternion(xx, yy, zz, a):
    factor = np.sin(a / 2.0)
    x = xx * factor
    y = yy * factor
    z = zz * factor
    w = np.cos(a / 2.0)
    quaternion = np.array([x, y, z, w])
    quaternion = quaternion / np.linalg.norm(quaternion)
    return quaternion


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def quaternion_inverse(quaternion):
    w, x, y, z = quaternion
    return np.array([w, -x, -y, -z])


def quaternion_to_base(input_quaternion, base_quaternion):
    # transform quaternion into base-frame
    return quaternion_multiply(quaternion_inverse(base_quaternion), input_quaternion)



