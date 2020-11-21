//
// Created by etienne on 21.11.20.
//

#include "normexp.h"


NormExp::NormExp() : expert_sampler_one_(config::control_dim){

	expert_sampler_one_.set_covariance(std::vector<double> {1,1});
	expert_sampler_one_.set_mean(std::vector<double> {0,0});

}

Eigen::MatrixXd NormExp::get_sample(size_t step, std::vector<double> state) {
	return expert_sampler_one_.get_sample();
}

void NormExp::update_expert(){
	// do nothing
}
