//
// Created by etienne on 03.11.20.
//

#include <vector>

namespace config
{
	int sim_time = 100;

	int rollouts = 100;
	int horizon = 20;

	std::vector<int> expert_types = {0, 1, 2};
	std::vector<int> expert_weights = {1, 1, 1};

	int obstacle_cost = 10000;
	std::vector<int> obstacle_rad = {10, 5, 10, 5};
	std::vector<int> obstacle_pos = {5, 20, 55, 15, 40, 10, 100, 40};

	extern std::vector<double> target_state = {100, 50, 0, 0, 0, 0};

	double pruning_threshold = 1.4;

	bool log_sampling = true;
}