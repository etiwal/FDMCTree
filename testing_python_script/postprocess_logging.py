import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import rospy
import sys
import os


def main(argv):

	assert(len(argv)==2)

	experiment_name = argv[0]
	experiment_id = argv[1]

	ee_desired_df = pd.read_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_end_effector_pose_desired_log.csv'))
	ee_real_df = pd.read_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_end_effector_log.csv'))
	cost_df = pd.read_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_cost_log.csv'))
	cost_min_df = pd.read_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_min_rollout_cost_log.csv'))
	cost_max_df = pd.read_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_max_rollout_cost_log.csv'))

	# cut time after timedelta
	time_delta = 60E9

	ee_desired_df = ee_desired_df.loc[ee_desired_df['time']<ee_desired_df['time'].min()+time_delta]
	ee_real_df = ee_real_df.loc[ee_real_df['time']<ee_real_df['time'].min()+time_delta]
	cost_df = cost_df.loc[cost_df['time']<cost_df['time'].min()+time_delta]
	cost_min_df = cost_min_df.loc[cost_min_df['time']<cost_min_df['time'].min()+time_delta]
	cost_max_df = cost_max_df.loc[cost_max_df['time']<cost_max_df['time'].min()+time_delta]


	#unify df

	unified_df = pd.merge_asof(ee_desired_df, ee_real_df, on='time', suffixes=('_DESI', '_REAL'), direction='nearest')
	unified_df = pd.merge_asof(unified_df, cost_df, on='time', direction='nearest')
	unified_df = pd.merge_asof(unified_df, cost_min_df, on='time', direction='nearest')
	unified_df = pd.merge_asof(unified_df, cost_max_df, on='time', direction='nearest')

	# add eucledian distance

	delta_x = unified_df['pos_x_DESI'] - unified_df['pos_x_REAL']
	delta_y = unified_df['pos_y_DESI'] - unified_df['pos_y_REAL']
	delta_z = unified_df['pos_z_DESI'] - unified_df['pos_z_REAL']

	unified_df['_eucledian_distance'] = np.sqrt( delta_x**2 + delta_y**2 + delta_z**2 )

	#split into seperate files
	unified_df['_x_compare'] = unified_df['pos_x_DESI']!=unified_df['pos_x_DESI'].shift(-1)
	unified_df['_y_compare'] = unified_df['pos_y_DESI']!=unified_df['pos_y_DESI'].shift(-1)
	unified_df['_z_compare'] = unified_df['pos_z_DESI']!=unified_df['pos_z_DESI'].shift(-1)

	unified_df['_desi_changed'] = unified_df[['_x_compare','_y_compare','_z_compare']].any(axis='columns')
	unified_df.loc[0, '_desi_changed'] = False

	unified_df['_seq'] = 0

	sequence=0
	for idx, row in unified_df.iterrows():
		if row['_desi_changed']==True:
			sequence+=1

		unified_df.loc[idx, '_seq'] = sequence


	# save df

	unified_df.to_csv(os.path.join('data_sim',experiment_name, experiment_name+'_'+experiment_id+'_unified.csv'))


	# save parts
	list_of_seq_elements = np.unique(unified_df['_seq'].to_list())
	print(list_of_seq_elements)
	for seq_idx in list_of_seq_elements:
		if seq_idx != 0:
			if not os.path.exists(os.path.join('data_sim', experiment_name, str(seq_idx))):
				os.makedirs((os.path.join('data_sim', experiment_name, str(seq_idx))))
		
			df_temp = unified_df.loc[unified_df['_seq']==seq_idx]
			df_temp.to_csv(os.path.join('data_sim',experiment_name, str(seq_idx), experiment_name+'_'+experiment_id+'_seq_'+str(seq_idx)+'_unified.csv'))

	return 0;

if __name__ == '__main__':
	main(sys.argv[1:])