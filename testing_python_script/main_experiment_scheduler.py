from threading import Thread
import os
from time import sleep
import sys


def main(argv):
	assert(len(argv)==2)

	experiment_name = argv[0]

	experiment_lenght = int(argv[1])

	if not os.path.exists(os.path.join('data_sim', experiment_name)):
		os.makedirs((os.path.join('data_sim', experiment_name)))

	for experiment_idx in range(0, experiment_lenght):
		print('Experiment Nr:', experiment_idx)
		os.system('python main_logger.py "' + str(experiment_name) + '" "' + str(experiment_lenght) + '_' + str(experiment_idx) + '"')
		os.system('python postprocess_logging.py "' + str(experiment_name) + '" "' + str(experiment_lenght) + '_' + str(experiment_idx) + '"')

if __name__ == '__main__':
	main(sys.argv[1:])