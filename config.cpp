//
// Created by etienne on 03.11.20.
//

#include <vector>
#include "config.h"

namespace config
{
	// Simulation
	int sim_time = 150;

	int rollouts = 20;
	int horizon = 20;

	// implementation
	bool use_cum_cost = true;
	bool use_last_best = true;

	// collision grid
	bool use_occupancy_grid = true;
	const char *grid_path = "/home/etienne/git_clone/FDMCTree/grid_python/60x100/60x100_complicated.csv";
	size_t grid_height = 60;
	size_t grid_width = 100;

	// - init
	std::vector<double> initial_state = {5, 5, 0, 0, 0, 0};
	std::vector<double> initial_mean = {0, 0};
	std::vector<double> initial_cov = {1, 1};

	// - target
	std::vector<double> target_state = {90, 40, 0, 0, 0, 0};

	int obstacle_cost = 10000;

	std::vector<int> obstacle_rad = {10, 5, 10, 5};
	std::vector<int> obstacle_pos = {5, 20, 55, 15, 40, 20, 80, 40};

	std::vector<int> expert_types = {0, 1, 2};
	std::vector<int> expert_weights = {1, 1, 1};

	double pruning_threshold = 0.5;

	// Robot
	int state_dim = 6;
	int control_dim = 2;

	double robot_mass = 10;

	// Logging
	bool log_sampling = true;

	// Debug
	size_t debug_level = 0;
}