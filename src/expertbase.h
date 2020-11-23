//
// Created by etienne on 21.11.20.
//

#ifndef MCSAMPLING_EXPERTBASE_H
#define MCSAMPLING_EXPERTBASE_H

#include <Eigen/Dense>
#include <vector>
#include <map>

#include "gaussian_sampler.h"
#include "config.h"


class ExpertBase{
public:
	ExpertBase() = default;
	~ExpertBase() = default;
	virtual void update_expert(Eigen::MatrixXd mean)=0;
	virtual Eigen::MatrixXd get_sample(size_t step, std::vector<double> state)=0;


//	std::map<size_t, GaussianSampler> experet_sampler_map;
//	GaussianSampler expert_sampler_one;
};

#endif //MCSAMPLING_EXPERTBASE_H
