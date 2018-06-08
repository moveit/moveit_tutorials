#!/usr/bin/env python
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
import geometry_msgs.msg
import time

class CreateScene(object):

    def __init__(self):
        self._scene = PlanningSceneInterface()

        # clear the scene
        self._scene.remove_world_object()
        self.robot = RobotCommander()

        # pause to wait for rviz to load
        rospy.sleep(4)

        box2_pose = [0.25, 0.25, 0.0, 0, 0, 0, 1]
        box2_dimensions = [0.25, 0.25, 0.45]
     
        self.add_box_object("box2", box2_dimensions, box2_pose)

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
	print "============ Waiting while RVIZ displays the scene with one obstacle..."
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("collision_scene_2")
    while not rospy.search_param('robot_description_semantic') and not rospy.is_shutdown():
        time.sleep(0.5)
    load_scene = CreateScene()
    rospy.spin()
