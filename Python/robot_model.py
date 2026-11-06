"""ECR5 modified-DH model.

Lengths are metres and joint angles are radians. The closed-form transform
matches ``roboticstoolbox`` ``RevoluteMDH`` / ``DHRobot.fkine`` / ``jacob0``.
"""

from math import pi

import numpy as np
import roboticstoolbox as rtb

# Link i stores Craig parameters (a_{i-1}, alpha_{i-1}, d_i, theta offset).
A = np.array([0.0, 0.150, 0.750, 0.155, 0.0, 0.0])
ALPHA = np.array([0.0, pi / 2, 0.0, pi / 2, -pi / 2, pi / 2])
D = np.array([0.253, 0.0, 0.0, 0.800, 0.0, 0.154])
OFFSET = np.array([0.0, pi / 2, 0.0, 0.0, -pi / 2, 0.0])


def make_robot():
    links = [
        rtb.RevoluteMDH(d=D[i], a=A[i], alpha=ALPHA[i], offset=OFFSET[i])
        for i in range(6)
    ]
    return rtb.DHRobot(links, name="ECR5")


def mdh_transform(a, alpha, d, theta):
    """One modified-DH link transform."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def fkine(q):
    """Homogeneous pose of the ECR5 tool at joint coordinate ``q`` (radians)."""
    pose = np.eye(4)
    frames = [pose]
    for i in range(6):
        pose = pose @ mdh_transform(A[i], ALPHA[i], D[i], q[i] + OFFSET[i])
        frames.append(pose)
    return pose, frames


def jacob0(q):
    """Geometric Jacobian in the base frame, matching ``DHRobot.jacob0``.

    Joint i of a modified-DH chain rotates about the z axis of frame i.
    """
    _, frames = fkine(q)
    origin = frames[-1][:3, 3]
    jacobian = np.zeros((6, 6))
    for i in range(6):
        axis = frames[i + 1][:3, 2]
        point = frames[i + 1][:3, 3]
        jacobian[:3, i] = np.cross(axis, origin - point)
        jacobian[3:, i] = axis
    return jacobian
