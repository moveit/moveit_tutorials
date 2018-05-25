import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time


class CreateScene1(object):
    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()

        self.robot = RobotCommander()

        # pause to wait for rviz to load
        rospy.sleep(4)

        floor_pose = [0, 0, -1.12, 0, 0, 0, 1]
        floor_dimensions = [4, 4, 0.02]

        box1_pose = [0.20, 0.50, 0.25, 0, 0, 0, 1]
        box1_dimensions = [0.2, 0.2, 0.5]

        box2_pose = [-0.55, -0.55, 0, 0, 0, 0, 1]
        box2_dimensions = [0.25, 0.25, 1.75]

        box3_pose = [0.5, -0.55, 0.14, 0, 0, 0, 1]
        box3_dimensions = [0.28, 0.28, 0.22]

        box4_pose = [-0.4, 0.4, 0.5, 0, 0, 0, 1]
        box4_dimensions = [0.25, 0.25, 1.1]

        self.add_box_object("floor", floor_dimensions, floor_pose)
        self.add_box_object("box1", box1_dimensions, box1_pose)
        self.add_box_object("box2", box2_dimensions, box2_pose)
        self.add_box_object("box3", box3_dimensions, box3_pose)
        self.add_box_object("box4", box4_dimensions, box4_pose)

    def add_box_object(self, name, dimensions, pose):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]
        self._scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))
        rospy.sleep(0.2)


if __name__ == "__main__":
    rospy.init_node("collision_scene_1")
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        time.sleep(0.5)
    load_scene = CreateScene1()
    rospy.spin()
