//
// Created by etienne on 27.10.20.
//

#include "fdmcts.h"

#include <iostream>
#include <random>
#include <ctime>


// Node object
//Node::Node(double state, double control_input, int step, int expert_type, size_t n, size_t parent) {
//	state_ = state;
//	control_input_ = control_input;
//	step_ = step;
//	expert_type_ = expert_type;
//	n_ = n;
//	parent_ = parent;
//
//	// call function to calc cost based on state
//	cost_ = get_cost(state_);
//}

Node::Node(const std::vector<double> &state, int step, size_t rollout, double cost_cum_parent) {
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

}

void Node::set_expert_type_manually(size_t expert_type){
	expert_type_ = expert_type;
}

void Node::sample_control_input(std::vector<double> state, int expert_type) {
	switch (expert_type) {
		// random sampling from uniform distribution
		case 0:
			control_input_ = {get_random_uniform_double(-1, 1), get_random_uniform_double(-1, 1)};
			break;
		case 1:
			control_input_ = {get_random_uniform_double(-2, 2), get_random_uniform_double(-2, 2)};
			break;
		case 2:
			control_input_ = {get_random_uniform_double(-10, 10), get_random_uniform_double(-10, 10)};
		default:
			control_input_ = {get_random_uniform_double(-1, 1), get_random_uniform_double(-1, 1)};;
	}

}


// Sys object
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


// Sim object


