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
	Node(double state, int step, int expert_type, size_t n, size_t parent);
	Node() = default;
	~Node() = default;

	double state_;
	double control_input_;
	int step_;
	int expert_type_;
	size_t n_;
	double cost_;
	double parent_;

	void set_control_input(double state, int expert_type);

	double get_state();

};

struct Sys{
public:
	Sys(double init_state);
	Sys() = default;
	~Sys() = default;

	double state_;

	void apply_control_input(double control_input);

	double get_state();
};
