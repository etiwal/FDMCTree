//
// Created by etienne on 03.11.20.
//

#include <vector>
#include "config.h"

namespace config
{
	// Simulation
	int sim_time = 200;

	int rollouts = 100;
	int horizon = 20;

	// implementation
	bool use_cum_cost = true;
	bool use_last_best = true;
	double pruning_threshold = 1;
	bool use_imp_sampling = false;
	bool use_cum_cost_for_imp_sampling = true;
//	double target_speed_cost_factor = 1;


	// collision grid
	bool use_occupancy_grid = true;
	const char *grid_path = "/home/etienne/git_clone/FDMCTree/grid_python/60x100/60x100_complicated_3borders.csv";
	size_t grid_height = 60;
	size_t grid_width = 100;

	// - init
	std::vector<double> initial_state = {5, 5, 0, 0, 0, 0};
	std::vector<double> initial_mean = {0, 0};
	std::vector<double> initial_cov = {1, 1};

	// - target
	std::vector<double> target_state = {87, 38, 0, 0, 0, 0};

	int obstacle_cost = 10000;

	std::vector<int> obstacle_rad = {10, 5, 10, 5};
	std::vector<int> obstacle_pos = {5, 20, 55, 15, 40, 20, 80, 40};

	std::vector<int> expert_types = {0, 1};
	std::vector<int> expert_weights = {1, 4};



	// Robot
	int state_dim = 6;
	int control_dim = 2;

	double robot_mass = 10;

	// Logging
	bool log_sampling = true;

	// Debug
	size_t debug_level = 0;
}