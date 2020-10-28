//
// Created by etienne on 27.10.20.
//

#ifndef MCSAMPLING_FDMCTS_H
#define MCSAMPLING_FDMCTS_H

#endif //MCSAMPLING_FDMCTS_H

#include <cstddef>
#include <vector>
#include <iostream>

class MCTree {
public:
	MCTree(size_t steps, size_t rollouts, size_t state_dim, size_t input_dim);
	MCTree() = default;
	~MCTree() = default;

	size_t steps_;
	size_t rollouts;
	size_t input_dim_;
	size_t state_dim_;
	double total_cost = 0.0;

private:
	void init_data();

public:
	void print_tree();

	void gen_child();


};

struct Node{
public:
	Node(double state, double control_input, int step, int expert_type, double cost=0);
	Node(double state, int step, int expert_type, double cost=0);
	Node() = default;
	~Node() = default;

	double state_;
	double control_input_;
	int step_;
	int expert_type_;
	double cost_;

	void set_control_input(double control_input);

	double get_state();


};
