#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker(number):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
	hello_str = "1711515 %d" % number
	number+=1
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    number=1
    try:
        talker(number)
    except rospy.ROSInterruptException:
        pass


