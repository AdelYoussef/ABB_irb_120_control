#!/usr/bin/env python
import rospy 
from moveit_python import  MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface
import tf.transformations #from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import MoveItErrorCodes, Grasp, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

from grasping_msgs.msg import FindGraspableObjectsAction
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation
import numpy as np

rospy.init_node("Test_pick_and_place")
franka1_panda_arm = MoveGroupInterface("franka1_panda_arm", "franka1_link0")

# create a planning scene interface, provide name of root link
franka1_scene = PlanningSceneInterface("franka1_link0")
franka2_scene = PlanningSceneInterface("franka2_link0")

# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
franka1_scene.addBox("table1", 0.2, 0.4, 0.4, 0.5, 0, 0.2)
franka1_scene.addBox("table2", 0.4, 0.2, 0.4, 0, -0.5, 0.2)

franka2_scene.addBox("table3", 0.2, 0.4, 0.4, 0.5, 0, 0.2)
franka2_scene.addBox("table4", 0.4, 0.2, 0.4, 0, 0.5, 0.2)

franka1_scene.addBox("cuboid1", 0.02, 0.02, 0.2, 0.5, 0, 0.5)
franka2_scene.addBox("cuboid2", 0.02, 0.02, 0.2, 0.5, 0, 0.5)


# do something
pick_place = PickPlaceInterface("franka1_panda_arm", "franka1_hand" )

grasps = Grasp()
# fill in franka1_grasps
#grasp pose
grasps.grasp_pose.header.frame_id = "franka1_link0"
orientation = tf.transformations.quaternion_from_euler(-3.14 / 2, -3.14 / 4, -3.14 / 2)
#print (orientation)
grasps.grasp_pose.pose.orientation.x = orientation[0]
grasps.grasp_pose.pose.orientation.y = orientation[1]
grasps.grasp_pose.pose.orientation.z = orientation[2]
grasps.grasp_pose.pose.orientation.w = orientation[3]
grasps.grasp_pose.pose.position.x = 0.415
grasps.grasp_pose.pose.position.y = 0
grasps.grasp_pose.pose.position.z = 0.5

#pre-grasp approach
grasps.pre_grasp_approach.direction.header.frame_id = "franka1_link0"
grasps.pre_grasp_approach.direction.vector.x = 1.0
grasps.pre_grasp_approach.direction.vector.y = 0.0
grasps.pre_grasp_approach.direction.vector.z = 0.0
grasps.pre_grasp_approach.min_distance = 0.095
grasps.pre_grasp_approach.desired_distance = 0.115

#post-grasp retreat
grasps.post_grasp_retreat.direction.header.frame_id = "franka1_link0" 
grasps.post_grasp_retreat.direction.vector.z = 1.0
grasps.post_grasp_retreat.direction.vector.y = 0.0
grasps.post_grasp_retreat.direction.vector.x = 0.0
grasps.post_grasp_retreat.min_distance = 0.1
grasps.post_grasp_retreat.desired_distance = 0.25

#open gripper
grasps.pre_grasp_posture.header.frame_id = "franka1_link0"
grasps.pre_grasp_posture.joint_names.append("franka1_finger_joint1") 
grasps.pre_grasp_posture.joint_names.append("franka1_finger_joint2")

pos = JointTrajectoryPoint()
pos.positions.append(0.04)
grasps.pre_grasp_posture.points.append(pos)
grasps.pre_grasp_posture.points.append(pos)

# time = JointTrajectoryPoint()
# time.time_from_start = rospy.rostime.Duration(0.5)
# grasps.pre_grasp_posture.points.append(time)

#close gripper
grasps.grasp_posture.joint_names.append("franka1_finger_joint1")
grasps.grasp_posture.joint_names.append("franka1_finger_joint2")

pos = JointTrajectoryPoint()
pos.positions.append(0.00)
grasps.grasp_posture.points.append(pos)
grasps.grasp_posture.points.append(pos)

# time = JointTrajectoryPoint()
# time.time_from_start = rospy.rostime.Duration(0.5)
# grasps.grasp_posture.points.append(time)
  
# print (grasps)
# setup object named object_name using PlanningSceneInterface
success = pick_place.pickup("cuboid1", [grasps], "table1")
print(success)

l = PlaceLocation()
# fill in l
l.place_pose.header.frame_id = "franka1_link0"
orientation = tf.transformations.quaternion_from_euler(0, 0, -3.14 / 2)
#print (orientation)
l.place_pose.pose.orientation.x = orientation[0]
l.place_pose.pose.orientation.y = orientation[1]
l.place_pose.pose.orientation.z = orientation[2]
l.place_pose.pose.orientation.w = orientation[3]
l.place_pose.pose.position.x = 0
l.place_pose.pose.position.y = -0.5
l.place_pose.pose.position.z = 0.5

l.pre_place_approach.direction.header.frame_id = "franka1_link0"
l.pre_place_approach.direction.vector.z = -1.0
l.pre_place_approach.min_distance = 0.095
l.pre_place_approach.desired_distance = 0.115

l.post_place_retreat.direction.header.frame_id = "franka1_link0"
l.post_place_retreat.direction.vector.y = 1.0
l.post_place_retreat.min_distance = 0.1
l.post_place_retreat.desired_distance = 0.25

#open gripper
l.post_place_posture.header.frame_id = "franka1_link0"
l.post_place_posture.joint_names.append("franka1_finger_joint1") 
l.post_place_posture.joint_names.append("franka1_finger_joint2")

pos = JointTrajectoryPoint()
pos.positions.append(0.04)
grasps.pre_grasp_posture.points.append(pos)
grasps.pre_grasp_posture.points.append(pos)

pick_place.place("cuboid1", [l], "table2", goal_is_eef = True)

#print ("Trial1")
# remove the cube
#franka1_scene.removeCollisionObject("cuboid1")
#franka2_scene.removeCollisionObject("cuboid2")
