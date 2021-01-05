#!/usr/bin/python

import pandas as pd
import numpy as np
import datetime
import time
import sys

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def create_msg(input_list):

	assert(len(input_list) == 7)	

	msg = PoseStamped()

	msg.pose.position.x = input_list[0]
	msg.pose.position.y = input_list[1]
	msg.pose.position.z = input_list[2]

	msg.pose.orientation.x = input_list[3]
	msg.pose.orientation.y = input_list[4]
	msg.pose.orientation.z = input_list[5]
	msg.pose.orientation.w = input_list[6]

	return msg

class Talker():
	def __init__ (self, topic):
		self.topic = topic
		self.pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

		rospy.init_node('talker_'+topic, anonymous=True)
		self.rate = rospy.Rate(10) 		

	def publish(self, msg, time_s):
		time_init_publisher = time.time()
		msg.header.frame_id = "world"		

		while (not rospy.is_shutdown()) & (time.time() < time_init_publisher+time_s):

			msg.header.stamp = rospy.Time.now()

			self.pub.publish(msg)
			self.rate.sleep()


def main(argv):

	assert(len(argv)==1)
	assert((argv[0]=='0') | (argv[0]=='1'))

	if argv[0] == '0':
		obstacle_position_df = pd.read_csv('positions_obstacle.txt', header=0)
		obst_publisher = Talker('obstacle')

		idx=0

		obst_msg = create_msg(obstacle_position_df.iloc[idx,:-1].values.tolist())
		try:	
			obst_publisher.publish(obst_msg, 2)	
		except rospy.ROSInterruptException:
			print("some error occured!")


	elif argv[0] == '1':
		ee_position_df = pd.read_csv('positions_ee.txt', header=0)

		ee_publisher = Talker('end_effector_pose_desired')

		for idx in range(0, ee_position_df.shape[0]):

			ee_msg = create_msg(ee_position_df.iloc[idx,:-1].values.tolist())
			try:
				ee_publisher.publish(ee_msg, int(ee_position_df['duration'].iloc[-1]))	  
			except rospy.ROSInterruptException:
				print("some error occured!")


if __name__ == '__main__':
	main(sys.argv[1:])
