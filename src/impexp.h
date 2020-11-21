//
// Created by etienne on 21.11.20.
//

#ifndef MCSAMPLING_IMPEXP_H
#define MCSAMPLING_IMPEXP_H

#include "expertbase.h"

class ImpExp : public ExpertBase{
public:
	ImpExp();
	~ImpExp() = default;

	Eigen::MatrixXd get_sample(size_t step, std::vector<double> state) override;

	void update_expert();

protected:
	GaussianSampler expert_sampler_one_;
	std::map<size_t, GaussianSampler> experet_sampler_map_;

};

#endif //MCSAMPLING_IMPEXP_H
