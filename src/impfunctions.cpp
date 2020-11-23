//
// Created by etienne on 23.11.20.
//

#include "impfunctions.h"

Eigen::ArrayXd get_omegas(Eigen::ArrayXd cost_vector){
	auto min_cost = cost_vector.minCoeff();
	auto max_cost = cost_vector.maxCoeff();

	if (min_cost==max_cost){
		max_cost=+0.0000000000000001;
	}

	cost_vector-1.0;

	auto h = 1.0;

	auto exponential_cost_ = Eigen::exp(-h * (cost_vector - min_cost)  / (max_cost - min_cost) );

	return exponential_cost_/exponential_cost_.sum();
}