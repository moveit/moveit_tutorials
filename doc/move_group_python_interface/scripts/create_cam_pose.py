import os
import numpy as np
import quaternion
import math


CENTER_POS = [0.55, 0.0, 0.04]
TARGET_POS1 = [
    [CENTER_POS[0], CENTER_POS[1], 0.55],
]
# working version - find better
TARGET_POS2 = [[0.17 * np.cos(x * np.pi / 180) + CENTER_POS[0], 0.21 * np.sin(x * np.pi / 180) + CENTER_POS[1], 0.44] for x in np.linspace(-160, 200, 50)]
TARGET_POS3 = [[0.17 * np.cos(x * np.pi / 180) + CENTER_POS[0], 0.25 * np.sin(x * np.pi / 180) + CENTER_POS[1], 0.39] for x in np.linspace(190, -160, 50)]
TARGET_POS4 = [[0.17 * np.cos(x * np.pi / 180) + CENTER_POS[0], 0.27 * np.sin(x * np.pi / 180) + CENTER_POS[1], 0.34] for x in np.linspace(-160, 120, 50)]
TARGET_POS = np.concatenate([TARGET_POS1, TARGET_POS2, TARGET_POS3, TARGET_POS4])
EE2CAM = [0.068915, 0.0325, 0] #numbers from cad

def look_at(x1, y1, z1, x2, y2, z2):
    """Function to look at x2, y2, z2 from x1, y1, z1"""
    target = np.array([x2, y2, z2])
    position = np.array([x1, y1, z1])
    forward = position - target
    forward = np.divide(forward, np.linalg.norm(forward))
    up = np.array([0, 0, -1])
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 0.001:
        epsilon = np.array([0.001, 0, 0])
        right = np.cross(forward, up + epsilon)
    right = np.divide(right, np.linalg.norm(right))

    up = np.cross(right, forward)
    up = np.divide(up, np.linalg.norm(up))
    rotmat = np.array([
        [right[0], up[0], -forward[0]],
        [right[1], up[1], -forward[1]],
        [right[2], up[2], -forward[2]],
    ])
    rotmat = np.matmul(rotmat, np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ]))

    # Additional rotating(45 degrees)for moveit config
    theta = np.deg2rad(45)
    adjustment = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    rotmat = np.matmul(rotmat, adjustment)
    return position, rotmat

if __name__ == "__main__":
    cam_positions = []
    cam_oris = []
    for pos in TARGET_POS:
        position, rotmat = look_at(pos[0], pos[1], pos[2], CENTER_POS[0], CENTER_POS[1], CENTER_POS[2])
        position = position - np.matmul(rotmat, EE2CAM)
        ori = quaternion.from_rotation_matrix(rotmat)
        # Change to numpy array[w, x, y, z]
        ori = np.array([ori.w, ori.x, ori.y, ori.z])
        cam_positions.append(position)
        cam_oris.append(ori)

    cam_positions = np.array(cam_positions)
    cam_oris = np.array(cam_oris)

    # Save to txt
    np.savetxt("positions.txt", cam_positions)
    np.savetxt("oris.txt", cam_oris)
