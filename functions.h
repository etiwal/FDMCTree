//
// Created by etienne on 13.10.20.
//

#ifndef MCSAMPLING_FUNCTIONS_H


#include <vector>
#include <random>
#include <boost/format.hpp>

#include "tree.h"

#include "config.h"

#include "gaussian_sampler.h"
#include "occupancygrid.h"



double get_cost(std::vector<double> state);

double get_random_uniform_double(double minV, double maxV);

unsigned get_random_uniform_unsigned(unsigned minV, unsigned maxV);

size_t get_expert_type(int rollout, size_t sampling_type);

void debug_print(size_t debug_lv, const boost::format& boost_str);

size_t get_unique_node_id(size_t sim_time, size_t horizon_step, size_t rollout, bool init);


#define MCSAMPLING_FUNCTIONS_H
#endif //MCSAMPLING_FUNCTIONS_H