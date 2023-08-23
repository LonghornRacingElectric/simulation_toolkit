import numpy as np


def rotation_x(angle, vector=None):
    matrix = np.array([[1, 0, 0],
                       [0, np.cos(angle), -np.sin(angle)],
                       [0, np.sin(angle), np.cos(angle)]])

    return matrix if (vector is None) else np.matmul(matrix, vector)


def rotation_y(angle, vector=None):
    matrix = np.array([[np.cos(angle), 0, np.sin(angle)],
                       [0, 1, 0],
                       [-np.sin(angle), 0, np.cos(angle)]])

    return matrix if (vector is None) else np.matmul(matrix, vector)


def rotation_z(angle, vector=None):
    matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                       [np.sin(angle), np.cos(angle), 0],
                       [0, 0, 1]])

    return matrix if (vector is None) else np.matmul(matrix, vector)
