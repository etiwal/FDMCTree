//
// Created by etienne on 10.11.20.
//

#include "sys_sim.h"
#include <vector>

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