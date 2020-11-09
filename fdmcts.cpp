//
// Created by etienne on 27.10.20.
//

#include "fdmcts.h"

#include <iostream>
#include <random>
#include <ctime>

// Expert
Expert::Expert(const std::vector<int>& expert_type_list) {
	expert_type_list_ = expert_type_list;
}

GaussianSampler Expert::get_expert_sampler(const std::vector<double>& state, size_t expert_type, const GaussianSampler& sampler_parent) {
	//assert() that the expert is in the list of experts!

	GaussianSampler expert_sampler(config::control_dim);

	switch (expert_type) {
		// Gauss random
		case 0:
			expert_sampler.set_covariance(std::vector<double> {0.5,0.5});
			expert_sampler.set_mean(std::vector<double> {0,0});
			break;
		case 1:
			expert_sampler.set_covariance(std::vector<double> {1,1});
			expert_sampler.set_mean(std::vector<double> {0,0});
			break;
		case 2:
			expert_sampler.set_covariance(std::vector<double> {2,2});
			expert_sampler.set_mean(std::vector<double> {0,0});
			// Gauss and informed by previous
			break;
		case 3:
			expert_sampler.set_covariance(std::vector<double> {0.5,0.5});
			expert_sampler.set_mean(std::vector<double> {0,0});

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

//Expert Expert_Instance(config::expert_types);


// Node
Node::Node(const std::vector<double> &state, int step, size_t rollout, double cost_cum_parent, GaussianSampler parent_sampler) : sampler_(2), 	parent_sampler_(2), Expert_Instance(config::expert_types) {
	state_ = state;
	step_ = step;

	rollout_ = rollout;
	control_input_ = {0, 0};

	cost_cum_parent_ = cost_cum_parent;

	// get the expert type depending on the index of the rollout from a LOT
	//YAML::Node ConfigNode = YAML::LoadFile("./config.yaml");
	expert_type_ = get_expert_type(rollout_, 0);

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

////legacy code!
//void Node::sample_control_input(std::vector<double> state, int expert_type) {
//	switch (expert_type) {
//		// random sampling from uniform distribution
//		case 0:
//			control_input_ = {get_random_uniform_double(-1, 1), get_random_uniform_double(-1, 1)};
//			break;
//		case 1:
//			control_input_ = {get_random_uniform_double(-2, 2), get_random_uniform_double(-2, 2)};
//			break;
//		case 2:
//			control_input_ = {get_random_uniform_double(-10, 10), get_random_uniform_double(-10, 10)};
//		case 3:
//			control_input_ = {get_random_uniform_double(-10, 10), get_random_uniform_double(-10, 10)};
//		default:
//			control_input_ = {get_random_uniform_double(-1, 1), get_random_uniform_double(-1, 1)};;
//	}
//
//}//legacy code!


// Sys
Sys::Sys(std::vector<double> &init_state) {
	state_ = init_state;
}

void Sys::apply_control_input(const std::vector<double>& control_input, int timesteps){
	//TODO: fix this!
	state_ = sim_system(state_, control_input, timesteps);
}

std::vector<double> Sys::get_state(){
	return state_;
}

int Sys::get_state_dim(){
	return 6;
};

int Sys::get_control_dim(){
	return 2;
};




