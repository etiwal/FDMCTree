//
// Created by etienne on 21.11.20.
//

#ifndef MCSAMPLING_NORMEXP_H
#define MCSAMPLING_NORMEXP_H

#include "expertbase.h"

class NormExp : public ExpertBase{
public:
	NormExp();
	~NormExp() = default;

	Eigen::MatrixXd get_sample(size_t step, std::vector<double> state) override;

protected:
	GaussianSampler expert_sampler_one_;

};

#endif //MCSAMPLING_NORMEXP_H
