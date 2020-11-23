//
// Created by etienne on 21.11.20.
//

#ifndef MCSAMPLING_EXPERT_H
#define MCSAMPLING_EXPERT_H

#include <cstddef>
#include <vector>
#include <iostream>
#include <map>

#include "functions.h"
#include "gaussian_sampler.h"

#include "normexp.h"
#include "impexp.h"


class Expert{
public:
	Expert();
	~Expert() = default;

	std::vector<int> expert_type_list_;

	int get_expert_from_LUT(size_t rollout);

	GaussianSampler get_expert_sampler(const std::vector<double>& state, size_t expert_type, const GaussianSampler& sampler_parent);

	Eigen::MatrixXd get_sample(size_t expert_type, size_t step, const std::vector<double>& state);

	void update_expert(size_t expert_type, Eigen::MatrixXd mean);

	void update_experts();

private:
	std::map<size_t, int> rollout_expert_map;
	static size_t get_expert_type(int rollout, size_t sampling_type);

	// create experts
	std::map<size_t, ExpertBase*> experts_;


};

#endif //MCSAMPLING_EXPERT_H
