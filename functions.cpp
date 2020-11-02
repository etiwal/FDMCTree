//
// Created by etienne on 13.10.20.
//
#include <iostream>
#include <cstdlib>
#include <vector>
#include <random>

#include "functions.h"


double get_cost(std::vector<double> state){
	double ref_x = 200;
	double ref_y = 50;
	double cost = std::abs(ref_x - state[0]) + std::abs(ref_y - state[1]) ;

	return cost;
}

double get_random_uniform_double(double minV, double maxV){
	std::default_random_engine::result_type seed = 0;
	static std::default_random_engine e(seed);
	std::uniform_real_distribution<double> u(minV, maxV);
	return u(e);
}

unsigned get_random_uniform_unsigned(unsigned minV, unsigned maxV){
	std::default_random_engine::result_type seed = 0;
	static std::default_random_engine e(seed);
	std::uniform_int_distribution<unsigned> u(minV, maxV);
	return u(e);
}

std::vector<double> sim_system(std::vector<double> state, std::vector<double> control_input, double timesteps){
	double a_x = control_input[0] + state[4];
	double a_y = control_input[1] + state[5];

	double v_x = a_x * timesteps + state[2];
	double v_y = a_y * timesteps + state[3];

	double p_x = a_x/2*timesteps*timesteps + state[0];
	double p_y = a_y/2*timesteps*timesteps + state[1];

	return {p_x, p_y, v_x, v_y, a_x, a_y};
}

//void print_tree(tree<Node> tree_input){
//
//};
