//
// Created by etienne on 03.11.20.
//

#ifndef MCSAMPLING_CONFIG_H
#define MCSAMPLING_CONFIG_H


#include <vector>
#include <string>

namespace config {
	// Simulation
	extern int sim_time;

	extern int rollouts;
	extern int horizon;

	// implementation
	extern bool use_cum_cost;
	extern bool use_last_best;

	// collision grid
	extern bool use_occupancy_grid;
	extern const char *grid_path;
	extern size_t grid_height;
	extern size_t grid_width;

	// - init
	extern std::vector<double> initial_state;
	extern std::vector<double> initial_mean;
	extern std::vector<double> initial_cov;

	// - target
	extern std::vector<double> target_state;

	extern std::vector<int> expert_types;
	extern std::vector<int> expert_weights;

	extern int obstacle_cost;
	extern std::vector<int> obstacle_rad;
	extern std::vector<int> obstacle_pos;

	extern double pruning_threshold;

	// Robot
	extern int state_dim;
	extern int control_dim;

	extern double robot_mass;

	// Logging
	extern bool log_sampling;

	// Debug
	extern size_t debug_level;
}


#endif //MCSAMPLING_CONFIG_H


