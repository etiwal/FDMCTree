//
// Created by etienne on 13.10.20.
//
#include <iostream>
#include <cstdlib>

#include <vector>
#include <random>

#include "functions.h"

OccupancyGrid Grid;

double get_cost(std::vector<double> state) {
	double ref_x = config::target_state[0];
	double ref_y = config::target_state[1];
	double cost = std::sqrt(pow(ref_x - state[0],2) + pow(ref_y - state[1], 2));

	double cost_obstacle = 0;
	if (config::use_occupancy_grid){
		if (Grid.check_position(state[0],state[1])){
			cost_obstacle += config::obstacle_cost;
		}
	} else {
		for (int i = 0; i < config::obstacle_pos.size(); i += 2) {
			// implementation for circular object
			double dist_from_obst_r;
			dist_from_obst_r = std::sqrt(
					pow(config::obstacle_pos[i] - state[0], 2) + pow(config::obstacle_pos[i + 1] - state[1], 2));
			if (dist_from_obst_r <= config::obstacle_rad[i - (i * 1 / 2)]) {
				cost_obstacle += config::obstacle_cost;
			}
		}
	}

	cost = cost + cost_obstacle;

	return cost;
}

double get_random_uniform_double(double minV, double maxV) {
	std::default_random_engine::result_type seed = 0;
	static std::default_random_engine e(seed);
	std::uniform_real_distribution<double> u(minV, maxV);
	return u(e);
}

unsigned get_random_uniform_unsigned(unsigned minV, unsigned maxV) {
	std::default_random_engine::result_type seed = 0;
	static std::default_random_engine e(seed);
	std::uniform_int_distribution<unsigned> u(minV, maxV);
	return u(e);
}

void debug_print(size_t debug_lv, const boost::format& boost_str){
	if (debug_lv <= config::debug_level){
		std::cout << boost_str.str() << std::endl;
	}
};

size_t get_unique_node_id(size_t sim_time, size_t horizon_step, size_t rollout, bool init){
	auto rootsize = 1;
	size_t unique_node_id = 0;

	unique_node_id += rootsize;

	if (!init){
		unique_node_id += config::rollouts+rootsize;
	}

	unique_node_id += (sim_time*config::horizon*config::rollouts) + (horizon_step*config::rollouts) + rollout;

	return unique_node_id;
}

