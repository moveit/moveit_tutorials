# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class TrajectoryGenerator(object):
    def __init__(self, positions, oris):
        super(TrajectoryGenerator, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_generator", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Class variables
        self.positions = positions
        self.oris = oris

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self, pos, ori):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = ori[0]
        pose_goal.orientation.x = ori[1]
        pose_goal.orientation.y = ori[2]
        pose_goal.orientation.z = ori[3]
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def plan(self, scale=1):
        move_group = self.move_group

        waypoints = []
        assert (len(self.oris) == len(self.positions))
        for _idx in range(len(self.positions)):
            pose_template = move_group.get_current_pose().pose
            pose_template.orientation.w = self.oris[_idx][0]
            pose_template.orientation.x = self.oris[_idx][1]
            pose_template.orientation.y = self.oris[_idx][2]
            pose_template.orientation.z = self.oris[_idx][3]
            pose_template.position.x = self.positions[_idx][0]
            pose_template.position.y = self.positions[_idx][1]
            pose_template.position.z = self.positions[_idx][2]
            waypoints.append(copy.deepcopy(pose_template))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 5.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        if fraction > 0.99: # 100% possible?
            print("================== Path planning complete! {}% planned ======================".format(fraction * 100))
            plan = self.move_group.retime_trajectory(
            self.robot.get_current_state(), plan, 0.1, 0.1)
            traj_length = len(plan.joint_trajectory.points)
            joint_traj = np.zeros((traj_length, 7))
            for _idx in range(traj_length):
                joint_traj[_idx] = plan.joint_trajectory.points[_idx].positions
            np.savetxt("joint_trajs.txt", joint_traj)
        else: 
            print("================== Could not plan whole path! {}% planned ======================".format(fraction * 100))
            # save to disk
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def retime_trajectory(self, plan, velocity_scaling=0.1, acceleration_scaling=0.1):
        plan = self.move_group.retime_trajectory(
            self.robot.get_current_state(), plan, velocity_scaling, acceleration_scaling
        )
        return plan


def main(positions, oris):
    Mover = TrajectoryGenerator(positions, oris)
    plan, fraction = Mover.plan()
    # Mover.display_trajectory(plan)
    plan = Mover.retime_trajectory(plan)
    Mover.execute_plan(plan)

if __name__ == "__main__":

    positions = np.loadtxt("positions.txt")
    oris = np.loadtxt("oris.txt")
    main(positions, oris)

    # # For graspnerf experiments
    # poses = np.loadtxt('/home/panda/libfranka/examples/trajectories/graspnerf/GraspNeRFPlanner_inmc_ss_1_grasp.txt')
    # pos = poses[12:15]
    # poses_reshape = poses.reshape(4, 4)
    # rotmat = poses_reshape[:3, :3]

    # # debug
    # baserot = np.array([
    #     [1, 0, 0],
    #     [0, -1, 0],
    #     [0, 0, -1]
    # ])
    # theta = np.deg2rad(45)
    # adjustment = np.array([
    #     [np.cos(theta), -np.sin(theta), 0],
    #     [np.sin(theta), np.cos(theta), 0],
    #     [0, 0, 1]
    # ])
    # rotmat = np.matmul(rotmat.T, adjustment)
 
    # r = R.from_matrix(rotmat)
    # ori = r.as_quat()
    # ori = np.array([ori[-1], ori[0], ori[1], ori[2]])
    # print(ori)
    # main(pos.reshape(1, 3), ori.reshape(1, 4))