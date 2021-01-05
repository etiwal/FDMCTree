import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def talker():
	pub = rospy.Publisher('end_effector_pose_desired', PoseStamped, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		msg = PoseStamped()
		msg.header.frame_id = "world"
		msg.header.stamp = rospy.Time.now()

		msg.pose.position.x = 0.5
		msg.pose.position.y = 0.0
		msg.pose.position.z = 0.22

		msg.pose.orientation.x = 0.5
		msg.pose.orientation.y = 0.0
		msg.pose.orientation.z = 0.22
		msg.pose.orientation.w = 0.5

		#rospy.loginfo(hello_str)
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
