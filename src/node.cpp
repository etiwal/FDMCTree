//
// Created by etienne on 27.10.20.
//

#include "node.h"

#include <iostream>
#include <random>
#include <ctime>

extern Expert Expert_Instance;


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







