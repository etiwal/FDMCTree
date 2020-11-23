//
// Created by etienne on 21.11.20.
//

#include "impexp.h"

ImpExp::ImpExp() : expert_sampler_one_(config::control_dim){
	expert_sampler_one_.set_covariance(std::vector<double> {1,1});
	expert_sampler_one_.set_mean(std::vector<double> {0,0});


	for (int i = 0; i < config::horizon; ++i) {
		experet_sampler_map_[i] = expert_sampler_one_;
	}
}

Eigen::MatrixXd ImpExp::get_sample(size_t step, std::vector<double> state) {
	return experet_sampler_map_[step].get_sample();
}

void ImpExp::update_expert(Eigen::MatrixXd mean) {
	for (int step = 0; step < mean.cols(); ++step) {
		experet_sampler_map_.at(step).set_mean(std::vector<double> {mean(0,step), mean(1,step)});
	}
	// calc exp weights etc
}

