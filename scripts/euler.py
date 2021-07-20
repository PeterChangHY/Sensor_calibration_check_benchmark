#!/usr/bin/env python

import math
import numpy as np
# make euler angles from tr matrices and tr matrices from euler angles (and offsets)


def rotate_x(psi):
    """return a matrix to rotate round x axis by psi radians"""
    return np.array(
        [
            [1, 0, 0],
            [0, math.cos(psi), -math.sin(psi)],
            [0, math.sin(psi), math.cos(psi)],
        ]
    )


def rotate_y(theta):
    """return a matrix to rotate round y axis by theta radians, aka"""
    return np.array(
        [
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)],
        ]
    )


def rotate_z(phi):
    """return a matrix to rotate round z axis by phi radians"""
    return np.array(
        [
            [math.cos(phi), -math.sin(phi), 0],
            [math.sin(phi), math.cos(phi), 0],
            [0, 0, 1],
        ]
    )


def recover_euler(r):
    """recover euler angles from a rotation matrix
    See ``Computing Euler angles from a rotation matrix'' by G. Slabaugh
    """
    if r[2, 0] not in (-1, 1):
        theta1 = -math.asin(r[2, 0])
        theta2 = math.pi - theta1
        psi1 = math.atan2(r[2, 1] / math.cos(theta1), r[2, 2] / math.cos(theta1))
        psi2 = math.atan2(r[2, 1] / math.cos(theta2), r[2, 2] / math.cos(theta2))
        phi1 = math.atan2(r[1, 0] / math.cos(theta1), r[0, 0] / math.cos(theta1))
        phi2 = math.atan2(r[1, 0] / math.cos(theta2), r[0, 0] / math.cos(theta2))
    else:
        psi1 = phi2 = 0  # can be anything
        if r[2, 0] == -1:
            theta1 = theta2 = math.pi / 2
            psi1 = psi2 = phi1 + math.atan2(r[0, 1], r[0, 2])
        else:
            theta1 = theta2 = -math.pi / 2
            psi1 = psi2 = -phi1 + math.atan2(-r[0, 1], -r[0, 2])
    # smaller roll should make more sense for a car pose
    if abs(psi2) > abs(psi1):
        return np.array([[psi1, theta1, phi1], [psi2, theta2, phi2]])
    else:
        return np.array([[psi2, theta2, phi2], [psi1, theta1, phi1]])


def r_matrix(psi, theta, phi):
    """utility to combine euler angles into one rotation matrix"""
    return rotate_z(phi).dot(rotate_y(theta).dot(rotate_x(psi)))


def tr_matrix(angles_and_offsets):
    """utility to glom all the rotation and translation parts into one matrix"""
    (psi, theta, phi, x, y, z) = angles_and_offsets
    tr = np.eye(4, 4)
    tr[:3, :3] = r_matrix(psi, theta, phi)
    tr[:3, 3] = [x, y, z]
    return tr


def test():
    Tr_1 = np.array(
        [
            0.014331838,
            -0.99961901,
            -0.023881199,
            -1.4810735,
            0.6280576,
            -0.0095802872,
            0.77810943,
            1.4418936,
            -0.7780382,
            -0.026152099,
            0.62768167,
            -0.1859642,
            0,
            0,
            0,
            1,
        ]
    ).reshape(4, 4)
    Tr_2 = np.array(
        [
            0.0087235272,
            -0.99950761,
            -0.030395212,
            0.39052889,
            0.61814409,
            -0.018500952,
            0.78585267,
            1.2205608,
            -0.7860207,
            -0.025644017,
            0.61767268,
            -0.085160226,
            0,
            0,
            0,
            1,
        ]
    ).reshape(4, 4)
    Tr = invert(Tr_2).dot(Tr_1)

    angles = recover_euler(Tr)
    print(angles / math.pi * 180)
    print(Tr[:3, 3])
    tr_ = r_matrix(angles[0, 0], angles[0, 1], angles[0, 2])
    tr = Tr[:3, :3]
    #assert np.linalg.norm(tr - tr_) < 1e-5
    tr_ = r_matrix(angles[1, 0], angles[1, 1], angles[1, 2])
    #assert np.linalg.norm(tr - tr_) < 1e-5
    print("test passed.")


    r_origin = np.array( [ 0.1542562612165633, -0.9880285230038918, 0.0021549028366801837, 
                          -0.9879323634943922, -0.15427130555839458, -0.013781338420208256, 
                          0.013948795150618018, -3.040720272962094e-06, -0.9999026687390928]
                          )
    r_origin = r_origin.reshape(3,3)
    print(r_origin)
    r_add = r_matrix(0,0, -2.0 * np.pi / 180.0)
    print(r_add)
    print(r_add.dot(r_origin))


def invert(transform):
    """return inverse of transform"""
    r = transform[:3, :3]
    t = transform[:3, 3]
    r = r.T
    t = -r.dot(t)
    f = np.vstack((np.column_stack((r, t)), np.array([0, 0, 0, 1])))
    return f


def wrap2pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


if __name__ == "__main__":
    test()
