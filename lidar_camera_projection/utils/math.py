import math
import numpy as np

def quaternion_to_rotation_matrix(x, y, z, w) -> np.array:
    xx = x * x
    xy = x * y
    xz = x * z
    xw = x * w

    yy = y * y
    yz = y * z
    yw = y * w

    zz = z * z
    zw = z * w

    rot_matrix = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - zw),     2 * (xz + yw)],
        [    2 * (xy + zw), 1 - 2 * (xx + zz),     2 * (yz - xw)],
        [    2 * (xz - yw),     2 * (yz + xw), 1 - 2 * (xx + yy)]
    ])

    return rot_matrix

def polar_to_xy(theta: int, r: int) -> np.array:
    return np.array([r * math.cos(theta), r * math.sin(theta)])

def xyz_to_homogenous(x, y, z):
    return np.array([x, y, z, 1]).reshape(4, 1)

def concatenate_translation(rotation_matrix: np.array, translation: np.array) -> np.array:
    translation_vector = translation.reshape(3, 1)
    transformation_matrix = np.hstack((rotation_matrix, translation_vector))
    transformation_matrix = np.vstack((transformation_matrix, [0, 0, 0, 1]))

    return transformation_matrix