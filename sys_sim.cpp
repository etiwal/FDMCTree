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
	state_ = sim_virtual_system(state_, control_input, timesteps);
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

std::vector<double> Sys::sim_virtual_system(std::vector<double> state, std::vector<double> control_input, double timesteps) {
	auto robot_mass = config::robot_mass;

	auto f_x = control_input[0];
	auto f_y = control_input[1];

	double a_x = f_x/robot_mass;
	double a_y = f_y/robot_mass;

	double v_x = (a_x * timesteps) + state[2];
	double v_y = (a_y * timesteps) + state[3];

	double p_x = (a_x / 2 * timesteps * timesteps) + (state[2] * timesteps) + state[0];
	double p_y = (a_y / 2 * timesteps * timesteps) + (state[3] * timesteps) + state[1];

	return {p_x, p_y, v_x, v_y, a_x, a_y};
}