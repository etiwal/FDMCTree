//
// Created by etienne on 13.10.20.
//
#include <iostream>
#include <cstdlib>
#include <vector>
#include <random>

#include "functions.h"


double get_cost(double state){
	double ref = 100;
	double cost = std::abs(ref - state);

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

double sim_system(double state, double control_input, double timesteps){
	return (state + (control_input * timesteps));
}

//void print_tree(tree<Node> tree_input){
//
//};
