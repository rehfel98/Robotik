#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, String


def talker():
	pub = rospy.Publisher('/assignment_publisher_subscriber', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %f" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I got that %f", data.data)
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/yaw", Float32, callback)
	rospy.spin()
if __name__ == '__main__':
	listener()


