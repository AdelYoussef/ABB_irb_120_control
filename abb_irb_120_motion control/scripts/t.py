#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64

def talker():
    pub = rospy.Publisher('/abb_irb120_3_58/joint_1_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        position = (1.0)
        pub.publish(position)
        rospy.loginfo(position)
        rate.sleep()

if __name__ == '__main__':
   try:
       talker()
   except rospy.ROSInterruptException:
       pass
