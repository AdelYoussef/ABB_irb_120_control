		hand_open()
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w = 0
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		grasping_group = 'gripper'
		touch_links = robot.get_link_names(group=grasping_group)
		scene.attach_box("gripper_leftfinger", "box", touch_links=touch_links)
		rospy.sleep(1)	
		hand_close()	
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0.2
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		hand_open()
		scene.remove_attached_object("gripper_leftfinger", name="box")
		rospy.sleep(1)
		pose_goal.position.x = 0.4
		pose_goal.position.y = 0
		pose_goal.position.z = 0.1
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()




		# fill in grasps
		#grasp pose
		grasps.grasp_pose.header.frame_id = "base_link"
		orientation = tf.transformations.quaternion_from_euler(0, 0, 0)
		#print (orientation)
		grasps.grasp_pose.pose.orientation.x = orientation[0]
		grasps.grasp_pose.pose.orientation.y = orientation[1]
		grasps.grasp_pose.pose.orientation.z = orientation[2]
		grasps.grasp_pose.pose.orientation.w = 0
		grasps.grasp_pose.pose.position.x = 0.415
		grasps.grasp_pose.pose.position.y = 0
		grasps.grasp_pose.pose.position.z = 0.5

		#pre-grasp approach
		grasps.pre_grasp_approach.direction.header.frame_id = "base_link"
		grasps.pre_grasp_approach.direction.vector.x = 1.0
		grasps.pre_grasp_approach.direction.vector.y = 0.0
		grasps.pre_grasp_approach.direction.vector.z = 0.0
		grasps.pre_grasp_approach.min_distance = 0.095
		grasps.pre_grasp_approach.desired_distance = 0.115

		#post-grasp retreat
		grasps.post_grasp_retreat.direction.header.frame_id = "base_link" 
		grasps.post_grasp_retreat.direction.vector.z = 1.0
		grasps.post_grasp_retreat.direction.vector.y = 0.0
		grasps.post_grasp_retreat.direction.vector.x = 0.0
		grasps.post_grasp_retreat.min_distance = 0.1
		grasps.post_grasp_retreat.desired_distance = 0.25
		
		#open gripper
		grasps.pre_grasp_posture.header.frame_id = "base_link"
		grasps.pre_grasp_posture.joint_names.append("gripper_finger_joint1") 
		grasps.pre_grasp_posture.joint_names.append("gripper_finger_joint2")
		pos = JointTrajectoryPoint()
		pos.positions.append(0.04)
		grasps.pre_grasp_posture.points.append(pos)
		grasps.pre_grasp_posture.points.append(pos)

		#close gripper
		grasps.grasp_posture.joint_names.append("gripper_finger_joint1")
		grasps.grasp_posture.joint_names.append("gripper_finger_joint2")
		pos = JointTrajectoryPoint()
		pos.positions.append(0.01)
		grasps.grasp_posture.points.append(pos)
		grasps.grasp_posture.points.append(pos)

		success = pick_place.pickup("cuboid1", [grasps], "table1")
		print(success)
		l = PlaceLocation()

		# fill in l
		l.place_pose.header.frame_id = "base_link"
		orientation = tf.transformations.quaternion_from_euler(0, 0, -pi/2)
		#print (orientation)
		l.place_pose.pose.orientation.x = orientation[0]
		l.place_pose.pose.orientation.y = orientation[1]
		l.place_pose.pose.orientation.z = orientation[2]
		l.place_pose.pose.orientation.w = 0
		l.place_pose.pose.position.x = 0
		l.place_pose.pose.position.y = -0.5
		l.place_pose.pose.position.z = 0.5

		l.pre_place_approach.direction.header.frame_id = "base_link"
		l.pre_place_approach.direction.vector.z = -1.0
		l.pre_place_approach.min_distance = 0.095
		l.pre_place_approach.desired_distance = 0.115

		l.post_place_retreat.direction.header.frame_id = "base_link"
		l.post_place_retreat.direction.vector.y = 1.0
		l.post_place_retreat.min_distance = 0.1
		l.post_place_retreat.desired_distance = 0.25
		
		#close gripper
		l.post_place_posture.header.frame_id = "base_link"
		l.post_place_posture.joint_names.append("gripper_finger_joint1") 
		l.post_place_posture.joint_names.append("gripper_finger_joint2")
		pos = JointTrajectoryPoint()
		pos.positions.append(0.04)
		grasps.pre_grasp_posture.points.append(pos)
		grasps.pre_grasp_posture.points.append(pos)

		pick_place.place("cuboid1", [l], "table2", goal_is_eef = True)
