#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker(t):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        #hello_str = "1711515 %s" % rospy.get_time()
	hello_str = "1711515 %d" % t
	t+=1
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    t=1
    try:
        talker(t)
    except rospy.ROSInterruptException:
        pass


