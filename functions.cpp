//
// Created by etienne on 13.10.20.
//
#include <iostream>
#include <cstdlib>
#include <vector>
#include <random>

#include "functions.h"


double get_cost(std::vector<double> state) {
	double ref_x = config::target_state[0];
	double ref_y = config::target_state[1];
	double cost = std::sqrt(pow(ref_x - state[0],2) + pow(ref_y - state[1], 2));

	double cost_obstacle = 0;
	for (int i = 0; i < config::obstacle_pos.size(); i+=2) {
		// implementation for circular object
		double dist_from_obst_r;
		dist_from_obst_r = std::sqrt(pow(config::obstacle_pos[i]-state[0],2)+pow(config::obstacle_pos[i+1]-state[1],2));
		if (dist_from_obst_r <= config::obstacle_rad[i-(i*1/2)]){
			cost_obstacle += config::obstacle_cost;
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

std::vector<double> sim_system(std::vector<double> state, std::vector<double> control_input, double timesteps) {
	auto robot_mass = config::robot_mass;

	auto f_x = control_input[0];
	auto f_y = control_input[1];

	double a_x = f_x/robot_mass;
	double a_y = f_y/robot_mass;

	double v_x = (a_x * timesteps) + state[2];
	double v_y = (a_y * timesteps) + state[3];

	double p_x = (a_x / 2 * timesteps * timesteps) + (state[2] * timesteps) + state[0];
	double p_y = (a_y / 2 * timesteps * timesteps) + (state[3] * timesteps) + state[1];

	return {p_x, p_y, v_x, v_y, a_x, a_y};
}

void debug_print(size_t debug_lv, const boost::format& boost_str){
	if (debug_lv <= config::debug_level){
		std::cout << boost_str.str() << std::endl;
	}
};