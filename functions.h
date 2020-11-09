//
// Created by etienne on 13.10.20.
//

#ifndef MCSAMPLING_FUNCTIONS_H
#define MCSAMPLING_FUNCTIONS_H

#endif //MCSAMPLING_FUNCTIONS_H

#include <vector>
#include <random>

#include "tree.h"

#include "config.h"

#include "gaussian_sampler.h"


double get_cost(std::vector<double> state);

double get_random_uniform_double(double minV, double maxV);

unsigned get_random_uniform_unsigned(unsigned minV, unsigned maxV);

std::vector<double> sim_system(std::vector<double> state, std::vector<double> control_input, double timesteps);

size_t get_expert_type(int rollout, size_t sampling_type);

GaussianSampler combine_distributions(std::vector<GaussianSampler> input_dists, size_t type);

//void print_tree(tree<Node> tree_input);