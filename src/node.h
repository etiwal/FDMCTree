//
// Created by etienne on 27.10.20.
//

#ifndef MCSAMPLING_NODE_H
#define MCSAMPLING_NODE_H

#include <cstddef>
#include <vector>
#include <iostream>
#include <map>

#include "functions.h"
#include "gaussian_sampler.h"
#include "expert.h"



struct Node{
public:
//	Node(double state, double control_input, int step, int expert_type, size_t n, size_t parent);
	Node(size_t node_id, const std::vector<size_t>& parent_node_id_path, const std::vector<double> &state, int step, size_t rollout, double cost_cum_parent, const GaussianSampler& parent_sampler);
	Node() = default;
	~Node() = default;

	std::vector<double> state_;
	std::vector<double> control_input_;
	int step_;
	int expert_type_;
	size_t rollout_;
	double cost_;
	double cost_cum_;
	double cost_cum_parent_;

	size_t node_id_;
	std::vector<size_t> parent_node_id_path_;
	std::vector<size_t> node_id_path_;

	void set_control_input(std::vector<double> control_input);

	void set_expert_type_manually(size_t expert_type);

	//TODO: Make it work with private object.
	GaussianSampler parent_sampler_;
	GaussianSampler sampler_;
};



#endif //MCSAMPLING_NODE_H