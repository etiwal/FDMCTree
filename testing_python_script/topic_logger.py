#!/usr/bin/env python
import rospy
import csv
import sys
import os
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped

class listener():
	def __init__(self, topic, experiment_name, experiment_id, msg_mode):
		self.filename = os.path.join('data_sim', experiment_name, experiment_name+'_'+experiment_id+'_'+topic+'_log.csv')

		rospy.init_node('listener_'+topic, anonymous=True)

		self.init_time = rospy.Time.now()


		if msg_mode=='PoseStamped':
			rospy.Subscriber(topic, PoseStamped, self.callback_PoseStamped)
			header_row = ['seq', 'time', 'pos_x', 'pos_y', 'pos_z', 'orient_x', 'orient_y', 'orient_z', 'orient_w']
		elif msg_mode=='double':
			rospy.Subscriber(topic, Float64, self.callback_double)
			if topic=='cost':
				header_row = ['time', 'cost']
			elif topic=='min_rollout_cost':
				header_row = ['time', 'min_rollout_cost']
			elif topic=='max_rollout_cost':
				header_row = ['time', 'max_rollout_cost']
			else:
				assert(1==0)
		else:
				assert(1==0)

		self.writerow(header_row, mode='wb')


		self.start_recording()


	def start_recording(self):
		while (self.init_time + rospy.Duration(80) > rospy.Time.now()):
			#print('INIT TIME:', self.init_time)
			#print('Threshold:', self.init_time + rospy.Duration(70))
			#print('NOW:', rospy.Time.now())
			rospy.sleep(100)

		print('Im done')


	def callback_PoseStamped(self, msg):
		seq = msg.header.seq
		time = msg.header.stamp

		pos_x = msg.pose.position.x
		pos_y = msg.pose.position.y
		pos_z = msg.pose.position.z

		orient_x = msg.pose.orientation.x
		orient_y = msg.pose.orientation.y
		orient_z = msg.pose.orientation.z
		orient_w = msg.pose.orientation.w

		row = [seq, time, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w]

		self.writerow(row, mode='ab')

	def callback_double(self, msg):
		time = rospy.Time.now()
		cost = msg.data

		row = [time, cost]

		self.writerow(row, mode='ab')

	def writerow(self, row, mode='ab'):
		with open(self.filename, mode) as csvfile:
			log_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			log_writer.writerow(row)

def main(argv):

	assert(len(argv)==3)
	assert((argv[0]=='0') | (argv[0]=='1') | (argv[0]=='2')| (argv[0]=='3')| (argv[0]=='4'))

	if argv[0]=='0':
		ee_pose_logger = listener('end_effector', argv[1], argv[2], msg_mode='PoseStamped')
	elif argv[0]=='1':
		ee_desired_pose_logger = listener('end_effector_pose_desired', argv[1], argv[2],msg_mode='PoseStamped')
	elif argv[0]=='2':
		cost_logger = listener('cost', argv[1], argv[2],msg_mode='double')
	elif argv[0]=='3':
		cost_min_rollout_logger = listener('min_rollout_cost', argv[1], argv[2],msg_mode='double')
	elif argv[0]=='4':
		cost_max_rollout_logger = listener('max_rollout_cost', argv[1], argv[2],msg_mode='double')


if __name__ == '__main__':
	main(sys.argv[1:])