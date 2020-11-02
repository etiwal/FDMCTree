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

Node::Node(std::vector<double> state, int step, int expert_type, size_t n) {
	state_ = state;
	step_ = step;
	expert_type_ = expert_type;
	n_ = n;
	control_input_ = {0, 0};

	// call function to calc cost based on state
	cost_ = get_cost(state_);

}

void Node::sample_control_input(std::vector<double> state, int expert_type) {
	control_input_ = {get_random_uniform_double(-1, 1), get_random_uniform_double(-1, 1)};
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


