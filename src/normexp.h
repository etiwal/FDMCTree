//
// Created by etienne on 21.11.20.
//

#ifndef MCSAMPLING_NORMEXP_H
#define MCSAMPLING_NORMEXP_H

#include "expertbase.h"
//#include "trajects.h"

class NormExp : public ExpertBase{
public:
	NormExp();
	~NormExp() = default;

	Eigen::MatrixXd get_sample(size_t step, std::vector<double> state) override;

	void update_expert(Eigen::MatrixXd mean);

protected:
	GaussianSampler expert_sampler_one_;

};

#endif //MCSAMPLING_NORMEXP_H
