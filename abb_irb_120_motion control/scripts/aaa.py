#! /usr/bin/env python
import sys
import rospy
import copy
import moveit_commander
import geometry_msgs.msg 
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

# ROS initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robotics_lab', anonymous=True)
motion_pub = rospy.Publisher("/robot_trajectory", geometry_msgs.msg.Pose, queue_size=1)

# Move-it initialization
robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("manipulator")
#display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

while not rospy.is_shutdown():

	command = int(input("enter command:	"))

	if command == 1 : #UP
		pose_target=geometry_msgs.msg.Pose()
		pose_target.orientation.w=0
		pose_target.position.x=0
		pose_target.position.y=0
		pose_target.position.z=pose_target.position.z+1
		group.set_pose_target(pose_target)

	elif command == 2 : #LEFT
		pose_target=geometry_msgs.msg.Pose()
		pose_target.orientation.w=0
		pose_target.position.x=pose_target.position.x+1
		pose_target.position.y=0
		pose_target.position.z=0
		group.set_pose_target(pose_target)
	
	elif command == 3 : #RIGHT
		pose_target=geometry_msgs.msg.Pose()
		pose_target.orientation.w=0
		pose_target.position.x=pose_target.position.x-1
		pose_target.position.y=0
		pose_target.position.z=0
		group.set_pose_target(pose_target)	

	elif command == 4: #DOWN
		pose_target=geometry_msgs.msg.Pose()
		pose_target.orientation.w=0
		pose_target.position.x=0
		pose_target.position.y=0
		pose_target.position.z=pose_target.position.z-1
		group.set_pose_target(pose_target)


	plan1=group.plan()
	rospy.sleep(3)
	motion_pub.publish(plan1.geometry_msgs.msg.Pose)
