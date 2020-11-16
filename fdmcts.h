//
// Created by etienne on 27.10.20.
//

#ifndef MCSAMPLING_FDMCTS_H
#define MCSAMPLING_FDMCTS_H

#endif //MCSAMPLING_FDMCTS_H

#include <cstddef>
#include <vector>
#include <iostream>
#include <map>

#include "functions.h"
#include "gaussian_sampler.h"

class Expert{
public:
	Expert();
	~Expert() = default;

	std::vector<int> expert_type_list_;

	int get_expert_from_LUT(size_t rollout);

	GaussianSampler get_expert_sampler(const std::vector<double>& state, size_t expert_type, const GaussianSampler& sampler_parent);

private:
	std::map<size_t, int> rollout_expert_map;
	static size_t get_expert_type(int rollout, size_t sampling_type);

};

//// defining Expert Object outside of Node
//Expert Expert_Instance;

struct Node{
public:
//	Node(double state, double control_input, int step, int expert_type, size_t n, size_t parent);
	Node(const std::vector<double> &state, int step, size_t rollout, double cost_cum_parent, const GaussianSampler& parent_sampler);
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

	void set_control_input(std::vector<double> control_input);

	void set_expert_type_manually(size_t expert_type);

	//TODO: Make it work with private object.
	GaussianSampler parent_sampler_;
	GaussianSampler sampler_;
	Expert Expert_Instance;
};


