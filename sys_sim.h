//
// Created by etienne on 10.11.20.
//

#ifndef MCSAMPLING_SYS_SIM_H
#define MCSAMPLING_SYS_SIM_H

#endif //MCSAMPLING_SYS_SIM_H

#include <vector>
#include "config.h"

struct Sys{
public:
	explicit Sys(std::vector<double> &init_state);
	Sys() = default;
	~Sys() = default;

	std::vector<double> state_;

	static int get_state_dim();

	static int get_control_dim();

	void apply_control_input(const std::vector<double>& control_input, int timesteps);

	std::vector<double> get_state();

	std::vector<double> sim_virtual_system(std::vector<double> state, std::vector<double> control_input, double timesteps);
};