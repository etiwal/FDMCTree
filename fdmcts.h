//
// Created by etienne on 27.10.20.
//

#ifndef MCSAMPLING_FDMCTS_H
#define MCSAMPLING_FDMCTS_H

#endif //MCSAMPLING_FDMCTS_H

#include <cstddef>
#include <vector>
#include <iostream>

#include "functions.h"


struct Node{
public:
//	Node(double state, double control_input, int step, int expert_type, size_t n, size_t parent);
	Node(std::vector<double> state, int step, int expert_type, size_t n);
	Node() = default;
	~Node() = default;

	std::vector<double> state_;
	std::vector<double> control_input_;
	int step_;
	int expert_type_;
	size_t n_;
	double cost_;

	void sample_control_input(std::vector<double> state, int expert_type);

};

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
};
