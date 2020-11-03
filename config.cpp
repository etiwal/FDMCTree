//
// Created by etienne on 03.11.20.
//

#include <vector>

namespace config
{
	int sim_time = 100;

	int rollouts = 20;
	int horizon = 10;

	std::vector<int> expert_types = {0, 1, 2};
	std::vector<int> expert_weights = {1, 1, 1};

	int obstacle_cost = 10000;
	std::vector<int> obstacle_rad = {5, 8, 5, 5};
	std::vector<int> obstacle_pos = {20, 10, 40, 30, 10, 25, 100, 40};

	extern std::vector<double> target_state = {100, 50, 0, 0, 0, 0};

	double pruning_threshold = 1.5;

	bool log_sampling = true;
}