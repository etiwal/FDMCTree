//
// Created by etienne on 27.10.20.
//

#include "fdmcts.h"
#include <iostream>

MCTree::MCTree(size_t horizon, size_t samples, size_t state_dim, size_t input_dim){
	MCTree::init_data();
};

void MCTree::init_data(){};

void MCTree::gen_child(){
	std::cout << "generating a child";
}







// Node struct/class

Node::Node(double state, double control_input, int step, int expert_type, double cost) {
	double state_ = state;
	double control_input_ = control_input;
	int step_ = step;
	int expert_type_ = expert_type;
	double cost_ = cost;
}

Node::Node(double state, int step, int expert_type, double cost) {
	double state_ = state;
	double control_input_ = 0;
	int step_ = step;
	int expert_type_ = expert_type;
	double cost_ = cost;
}

void Node::set_control_input(double control_input) {
	control_input_ = control_input;
}

double Node::get_state() {
	return state_;
}





