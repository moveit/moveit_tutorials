import os
import numpy as np
import math
import matplotlib.pyplot as plt

if __name__ == "__main__":
    assert os.path.exists('./joint_trajs.txt')
    joint_traj = np.loadtxt("joint_trajs.txt") # a N x 7 joint trajectory
    N = joint_traj.shape[0]
    velocities = np.zeros((N - 1, 7))
    velocities = joint_traj[1:] - joint_traj[:-1]
    accels = velocities[1:] - velocities[:-1]
    final_path = []

    # visualization
    angle_accel = np.sum(accels[1:] * accels[:-1], axis=-1) / (np.linalg.norm(accels[1:]) * np.linalg.norm(accels[:-1]))
    plt.plot(angle_accel)
    plt.show()
    breakpoint()

    # pad at start - assume we are at first start position
    start_points = []
    for t in np.linspace(0, np.pi / 2, 100):
        waypoint = joint_traj[0] + (velocities[0]) * np.sin(t)
        start_points.append(waypoint)

    # pad at end
    end_points = []
    for t in np.linspace(0, np.pi / 2, 100):
        waypoint = joint_traj[N - 1] + (velocities[-1]) * np.cos(t)
        end_points.append(waypoint)

    # joined path
    final_path = start_points + joint_traj[1: -1].tolist() + end_points
   
    # save to disk
    np.savetxt('joint_trajs_touched.txt', np.array(final_path))

