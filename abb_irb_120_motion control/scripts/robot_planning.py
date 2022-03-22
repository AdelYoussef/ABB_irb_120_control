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
from std_msgs.msg import String , Float64MultiArray
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from grasping_msgs.msg import FindGraspableObjectsAction
from moveit_msgs.msg import MoveItErrorCodes, PlaceLocation, Grasp
import numpy as np
from moveit_python import  MoveGroupInterface,PickPlaceInterface
import tf.transformations
from trajectory_msgs.msg import JointTrajectoryPoint

# ROS initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_planning', anonymous=True)
motion_pub = rospy.Publisher("/abb_controller/command", JointTrajectory, queue_size=1)
motion_pub1 = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)


# Move-it initialization
robot= moveit_commander.RobotCommander()
scene= moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("manipulator")
grip=moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
abb_arm = MoveGroupInterface("manipulator", "base_link")
pick_place = PickPlaceInterface("manipulator", "gripper" )
grasps = Grasp()
# set planning reference frame
group.set_pose_reference_frame("base_link")



# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""



scale_UD = 0
scale_LR = 0.4



def up ():


	global scale_UD 
	global scale_LR 
	scale_UD += 0.1
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = 0
	pose_goal.position.y = 0
	pose_goal.position.z = 0.86
	group.set_pose_target(pose_goal)
	plan=group.plan()
	motion_pub.publish(plan.joint_trajectory)
	#plan = group.go(wait=True)
	#group.stop()
	#group.clear_pose_targets()
	#rospy.loginfo(group.get_current_pose())
def down ():
	global scale_UD 
	global scale_LR 
	scale_UD -= 0.1
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = scale_LR
	pose_goal.position.y = scale_UD
	pose_goal.position.z = 0.1
	group.set_pose_target(pose_goal)
	plan=group.plan()
	motion_pub.publish(plan.joint_trajectory)
	# plan = group.go(wait=True)
	# group.stop()
	# group.clear_pose_targets()

def right ():
	global scale_UD 
	global scale_LR 
	scale_LR -= 0.1
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = scale_LR
	pose_goal.position.y = scale_UD
	pose_goal.position.z = 0.1
	group.set_pose_target(pose_goal)
	plan=group.plan()
	motion_pub.publish(plan.joint_trajectory)
	# plan = group.go(wait=True)
	# group.stop()
	# group.clear_pose_targets()

def left ():
	global scale_UD 
	global scale_LR 
	scale_LR += 0.1
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = scale_LR
	pose_goal.position.y = scale_UD
	pose_goal.position.z = 0.1	
	group.set_pose_target(pose_goal)
	plan=group.plan()
	motion_pub.publish(plan.joint_trajectory)
	# plan = group.go(wait=True)
	# group.stop()
	# group.clear_pose_targets()

	
def hand_open ():
	joint_goal = grip.get_current_joint_values()
	joint_goal[0]=0.04
	joint_goal[1]=0.04
	plan=grip.plan(joint_goal)
	motion_pub1.publish(plan.joint_trajectory)
	# group.execute(plan, wait=True)
	# grip.stop()

def hand_close ():
	joint_goal = grip.get_current_joint_values()
	joint_goal[0]=0.01
	joint_goal[1]=0.01
	plan=grip.plan(joint_goal)
	motion_pub1.publish(plan.joint_trajectory)
	# grip.go(joint_goal , wait=True)
	# grip.stop()
def box():
	data = rospy.wait_for_message('centers_rgb',Float64MultiArray)
	rospy.loginfo(data.data)
	joint_goal = grip.get_current_joint_values()

	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 1.0
	pose_goal.position.x = data.data[0]
	pose_goal.position.y = data.data[1]
	pose_goal.position.z = data.data[2]
	group.set_pose_target(pose_goal)
	plan=group.plan()
	motion_pub.publish(plan.joint_trajectory)
	rospy.sleep(2)



while not rospy.is_shutdown():

	command = int(input("enter command:	"))

	if command == 1 : #UP
		up()

	elif command == 2 : #LEFT
		left()
	
	elif command == 3 : #RIGHT
		right()	

	elif command == 4: #DOWN
		down()

	elif command == 5: #gripper
		hand_open()
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 1.0
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0
		pose_goal.position.z = 0.5
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		grasping_group = 'gripper'
		touch_links = robot.get_link_names(group=grasping_group)
		scene.attach_box("gripper_leftfinger", "cuboid", touch_links=touch_links)
		rospy.sleep(1)	
		hand_close()
		rospy.sleep(1)
		pose_goal.orientation.w = 0	
		pose_goal.position.x = 0.0
		pose_goal.position.y = -0.5
		pose_goal.position.z = 0.5
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		hand_open()
		scene.remove_attached_object("gripper_leftfinger", name="cuboid")
		rospy.sleep(1)
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0
		pose_goal.position.z = 0.5
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()

	elif command == 6: #gripper
		box()
