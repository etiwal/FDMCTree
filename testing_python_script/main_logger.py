from threading import Thread
import os
from time import sleep
import sys


def start_publisher(mode):
	print('creating publisher:', mode)
	cmd_string = " ".join(["python pose_publisher.py", str(mode)])
	os.system(cmd_string)

def start_logger(mode, experiment_name, experiment_id):
	print('creating logger:', mode)
	cmd_string = " ".join(["python topic_logger.py", str(mode), experiment_name, experiment_id])
	os.system(cmd_string)


def main(argv):

	experiment_name = argv[0]

	experiment_id = argv[1]

	thread_handles = []

	for mode in [0,1]:
		thread = Thread(target = start_publisher, args = (mode,))
		thread.start()
		thread_handles.append(thread)

	sleep(3)

	for mode in range(0,5):
		thread = Thread(target = start_logger, args = (mode, experiment_name, experiment_id))
		thread.start()
		thread_handles.append(thread)


	all_threads_killed=False
	while (all_threads_killed==False):
		if all([thread_handle.is_alive() is False for thread_handle in thread_handles]):
			print("now i can kill all threads!")
			all_threads_killed=True


	return 0


if __name__ == '__main__':
	main(sys.argv[1:])