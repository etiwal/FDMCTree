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
	double a_x = control_input[0] + state[4];
	double a_y = control_input[1] + state[5];

	double v_x = a_x * timesteps + state[2];
	double v_y = a_y * timesteps + state[3];

	double p_x = a_x / 2 * timesteps * timesteps + state[0];
	double p_y = a_y / 2 * timesteps * timesteps + state[1];

	return {p_x, p_y, v_x, v_y, a_x, a_y};
}

size_t get_expert_type(int rollout, size_t sampling_type) {
	size_t expert_type_ = 0;

	auto n_rollouts = config::rollouts;//ConfigNode["solver"]["rollouts"].as<int>();
	auto expert_types = config::expert_types;//ConfigNode["solver"]["experts"]["types"].as<std::vector<int>>();
	auto expert_weights = config::expert_weights;//ConfigNode["solver"]["experts"]["weights"].as<std::vector<double>>();

	assert(expert_types.size() == expert_weights.size());
	assert(sampling_type == 0 | sampling_type == 1);

	size_t n_experts = expert_weights.size();

	double sum_expert_weights = 0;
	for (double expert_weight : expert_weights) {
		sum_expert_weights += expert_weight;
	}

	std::vector<double> expert_weights_normalized_CDF(expert_weights.size());
	for (int i = 0; i < expert_weights.size(); ++i) {
		if (i==0){
			expert_weights_normalized_CDF[i] = expert_weights[i] / sum_expert_weights;
		} else {
			expert_weights_normalized_CDF[i] = expert_weights_normalized_CDF[i-1] + (expert_weights[i] / sum_expert_weights);
		}
	}

	// def for LOT
	double pos_rollout = (double) rollout / (double) n_rollouts;
	//TODO: Move this such that it is only created once and then directly called!
	switch (sampling_type) {
		// LOT (The weights of the different experts are guaranteed to be represented in the final round)
		case 0:
			for (int i = 0; i < expert_weights.size(); ++i) {
				// lower end
				if (i == 0) {
					if (pos_rollout < expert_weights_normalized_CDF[i]) {
						expert_type_ = expert_types[i];
						break;
					}
				} else { // other cases
					if (expert_weights_normalized_CDF[i-1] <= pos_rollout & pos_rollout < expert_weights_normalized_CDF[i]) {
						expert_type_ = expert_types[i];
						break;
					}
				}

			}
			break;

			// Sampling based on weights
		case 1:
			expert_type_ = expert_types[0];
			break;
	}

	return expert_type_;
}

//void print_tree(tree<Node> tree_input){
//
//};
