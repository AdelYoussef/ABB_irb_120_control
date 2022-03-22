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
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# ROS initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robotics_lab', anonymous=True)
#motion_pub = rospy.Publisher("/robot_trajectory", JointTrajectory, queue_size=1)

# Move-it initialization
robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

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
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0
pose_goal.position.z = 0.1
group.set_pose_target(pose_goal)
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
#group.execute(plan, wait=True)

while not rospy.is_shutdown():

	command = int(input("enter command:	"))

	if command == 1 : #UP
		scale_UD += 0.1
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.0
		pose_goal.position.x = scale_LR
		pose_goal.position.y = scale_UD
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)

	elif command == 2 : #LEFT
		scale_LR += 0.1
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.0
		pose_goal.position.x = scale_LR
		pose_goal.position.y = scale_UD
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)
	
	elif command == 3 : #RIGHT
		scale_LR -= 0.1
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.0
		pose_goal.position.x = scale_LR
		pose_goal.position.y = scale_UD
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)	

	elif command == 4: #DOWN
		scale_UD -= 0.1
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0.0
		pose_goal.position.x = scale_LR
		pose_goal.position.y = scale_UD
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)

	plan = group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	group.clear_pose_targets()
	#group.execute(plan, wait=True)

