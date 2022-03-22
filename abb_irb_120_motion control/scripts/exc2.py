#! /usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import pi

# ROS initialization
rospy.init_node('robotics_lab', anonymous=True)
# moveit initialization
group = MoveGroupCommander("manipulator")
rospy.sleep(1)
# set robot initial state
robot_state = RobotState()
robot_state.joint_state.header.frame_id = "base_link"
robot_state.joint_state.header.stamp = rospy.Time.now()
robot_state.joint_state.name = [ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
initial_state = [0, 0, 0, 0, 0, 0]
for i in range(6):
    robot_state.joint_state.position.append(initial_state[i] * (pi/180))
group.set_start_state(robot_state)
# set planning reference frame
group.set_pose_reference_frame("base_link")
# define joint goal
joint_goal = JointState()
joint_goal.header.frame_id = "base_link"
joint_goal.header.stamp = rospy.Time.now()
joint_goal.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5","joint_6"]
joint_goal.position = [
    90 * (pi / 180), -40 * (pi / 180), 0 * (pi / 180), 0 * (pi / 180),
    0 * (pi / 180), 0 * (pi / 180)]
    # motion planning to the required goal
plan = group.plan(joint_goal)
rospy.sleep(1)
