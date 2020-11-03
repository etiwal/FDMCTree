//
// Created by etienne on 03.11.20.
//

#ifndef MCSAMPLING_CONFIG_H
#define MCSAMPLING_CONFIG_H


#include <vector>

namespace config
{
	extern int sim_time;

	extern int rollouts;
	extern int horizon;

	extern std::vector<int> expert_types;
	extern std::vector<int> expert_weights;

	extern int obstacle_cost;
	extern std::vector<int> obstacle_rad;
	extern std::vector<int> obstacle_pos;

	extern std::vector<double> target_state;

	extern double pruning_threshold;

	extern bool log_sampling;
}


#endif //MCSAMPLING_CONFIG_H


