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

Node::Node(double state, int step, int expert_type, size_t n, size_t parent) {
	state_ = state;
	step_ = step;
	expert_type_ = expert_type;
	n_ = n;
	parent_ = parent;
	control_input_ = 0;

	// call function to calc cost based on state
	cost_ = get_cost(state_);

	// based on the state and on the expert type a control input is sampled
//	control_input_ = get_random_uniform_double(-5, 5);
}

void Node::set_control_input(double state, int expert_type) {
	control_input_ = get_random_uniform_double(-10, 10);
}


// Sys object
Sys::Sys(double init_state) {
	state_ = init_state;
}

void Sys::apply_control_input(double control_input) {
	state_ += control_input;
}

double Sys::get_state() {
	return state_;
}


// Sim object


