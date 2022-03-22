#! /usr/bin/env python
import rospy
from moveit_commander import  MoveGroupCommander
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

# ROS initialization
rospy.init_node('robotics_lab', anonymous=True)
motion_pub = rospy.Publisher("/robot_trajectory", JointTrajectory, queue_size=1)

rospy.sleep(1)

# moveit initialization
group = MoveGroupCommander("manipulator")
rospy.sleep(1)

# set planning reference frame
group.set_pose_reference_frame("base_link")

# set robot initial state
robot_state = RobotState()
robot_state.joint_state.header.frame_id = "base_link"
robot_state.joint_state.header.stamp = rospy.Time.now()
robot_state.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

scale_UP = 0
scale_LEFT = 0
scale_RIGHT = 0
scale_DOWN = 0

while not rospy.is_shutdown():

	#Subscribes BCI command
	command = int(input("enter command:	"))

	robot_state.joint_state.position = []
	joint_state_msg = rospy.wait_for_message("/robot_current_state", JointState)
	rospy.sleep(2)
	rospy.loginfo(joint_state_msg)

	if command == 1 : #UP
		scale_UP += 2
		joints = [joint_state_msg.position[0] * (pi/180),
joint_state_msg.position[1] * (pi/180),
joint_state_msg.position[2] * (pi/180)-scale_UP,
joint_state_msg.position[3] * (pi/180),
joint_state_msg.position[4] * (pi/180),
joint_state_msg.position[5] * (pi/180)]

	elif command == 2 : #LEFT
		scale_LEFT += 2
		joints = [joint_state_msg.position[0] * (pi/180)+scale_LEFT,
joint_state_msg.position[1] * (pi/180),
joint_state_msg.position[2] * (pi/180),
joint_state_msg.position[3] * (pi/180),
joint_state_msg.position[4] * (pi/180),
joint_state_msg.position[5] * (pi/180)]
		

	elif command == 3 : #RIGHT
		scale_RIGHT += 2
		joints = [joint_state_msg.position[0] * (pi/180)-scale_RIGHT,
joint_state_msg.position[1] * (pi/180),
joint_state_msg.position[2] * (pi/180),
joint_state_msg.position[3] * (pi/180),
joint_state_msg.position[4] * (pi/180),
joint_state_msg.position[5] * (pi/180)]

	elif command == 4: #DOWN
		scale_DOWN += 2
		joints = [joint_state_msg.position[0] * (pi/180),
joint_state_msg.position[1] * (pi/180),
joint_state_msg.position[2] * (pi/180)+scale_DOWN,
joint_state_msg.position[3] * (pi/180),
joint_state_msg.position[4] * (pi/180),
joint_state_msg.position[5] * (pi/180)]


#	else: #STOP
#		for i in range(6):
#			joints = joint_state_msg.position[i] * (pi/180)


	#set start state
	for i in range(6):
		  robot_state.joint_state.position.append(joint_state_msg.position[i] * (pi/180))

	group.set_start_state(robot_state)

	# define joint goal
	joint_goal = JointState()
	joint_goal.header.frame_id = "base_link"
	joint_goal.header.stamp = rospy.Time.now()
	joint_goal.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

	for i in range(6):
		  joint_goal.position.append(joints[i] * (pi/180))

	# motion planning to the required goal
	plan = group.plan(joint_goal)

	#publish joint trajectory plan
	if len(plan.joint_trajectory.points) > 1:
		  motion_pub.publish(plan.joint_trajectory)
	else:
		  rospy.loginfo("WARNING : no plan found")
