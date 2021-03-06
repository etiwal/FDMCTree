//
// Created by etienne on 27.10.20.
//

#include "fdmcts.h"

#include <iostream>
#include <random>
#include <ctime>

// Expert
Expert::Expert() {
	expert_type_list_ = config::expert_types;
	for (size_t i = 0; i < config::rollouts; ++i) {
		rollout_expert_map[i] = get_expert_type(i,0);
	}
}

GaussianSampler Expert::get_expert_sampler(const std::vector<double>& state, size_t expert_type, const GaussianSampler& sampler_parent) {
	//assert() that the expert is in the list of experts!

	GaussianSampler expert_sampler(config::control_dim);

	switch (expert_type) {
		// Gauss random
		case 0:
			expert_sampler.set_covariance(std::vector<double> {1,1});
			expert_sampler.set_mean(std::vector<double> {0,0});

			break;
		case 1:
			expert_sampler.set_covariance(std::vector<double> {2,2});
			expert_sampler.set_mean(std::vector<double> {0,0});
			break;
		case 2:
			expert_sampler.set_covariance(std::vector<double> {10,10});
			expert_sampler.set_mean(std::vector<double> {0,0});
			break;
		// Gauss and informed by previous
		case 3:
			static std::mt19937 gen{ std::random_device{}() };
			static std::normal_distribution<double> ann;

			expert_sampler.set_covariance(std::vector<double> {2,2});
			expert_sampler.set_mean(std::vector<double> {ann(gen),ann(gen)});

			expert_sampler.combine_dist_mult(sampler_parent);
			break;
		case 4:
			expert_sampler.set_covariance(std::vector<double> {4,4});
			expert_sampler.set_mean(std::vector<double> {0,0});

			expert_sampler.combine_dist_mult(sampler_parent);
			break;
		default:
			expert_sampler.set_covariance(std::vector<double> {1,1});
			expert_sampler.set_mean(std::vector<double> {0,0});
			break;
	}

	return expert_sampler;
}

int Expert::get_expert_from_LUT(size_t rollout){
	return rollout_expert_map.at(rollout);
}

size_t Expert::get_expert_type(int rollout, size_t sampling_type) {
	size_t expert_type_to_return = 0;

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
						expert_type_to_return = expert_types[i];
						break;
					}
				} else { // other cases
					if (expert_weights_normalized_CDF[i-1] <= pos_rollout & pos_rollout < expert_weights_normalized_CDF[i]) {
						expert_type_to_return = expert_types[i];
						break;
					}
				}

			}
			break;

			// Sampling based on weights
		case 1:
			perror("this type of expert sampling is not yet implemented");
			expert_type_to_return = expert_types[0];
			break;
	}

	return expert_type_to_return;
}


//// defining Expert Object outside of Node
Expert Expert_Instance;

// Node
Node::Node(size_t node_id, const std::vector<size_t>& parent_node_id_path, const std::vector<double> &state, int step, size_t rollout, double cost_cum_parent, const GaussianSampler& parent_sampler) : sampler_(2), parent_sampler_(2){
	state_ = state;
	step_ = step;

	node_id_ = node_id;
	parent_node_id_path_ = parent_node_id_path;

	node_id_path_ = parent_node_id_path_;
	node_id_path_.push_back(node_id_);

	rollout_ = rollout;
	control_input_ = {0, 0};

	cost_cum_parent_ = cost_cum_parent;

	if (config::use_last_best == true && rollout == 0 && step == config::horizon-1){
		auto temp_rollout = get_random_uniform_unsigned(0, config::rollouts-1);
		expert_type_ = Expert_Instance.get_expert_from_LUT(temp_rollout);
	} else {
		expert_type_ = Expert_Instance.get_expert_from_LUT(rollout_);
	}

	// call function to calc cost based on state
	cost_ = get_cost(state_);
	cost_cum_ = cost_cum_parent_ + cost_;

	parent_sampler_ = parent_sampler;

	// set intial mean and covariance
	sampler_ = Expert_Instance.get_expert_sampler(state_, expert_type_, parent_sampler_);
}

void Node::set_expert_type_manually(size_t expert_type){
	expert_type_ = expert_type;
}

void Node::set_control_input(std::vector<double> control_input) {
	control_input_ = control_input;
}







