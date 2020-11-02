//
// Created by etienne on 13.10.20.
//

#ifndef MCSAMPLING_FUNCTIONS_H
#define MCSAMPLING_FUNCTIONS_H

#endif //MCSAMPLING_FUNCTIONS_H

#include <vector>
#include <random>

#include "tree.h"



double get_cost(std::vector<double> state);

double get_random_uniform_double(double minV, double maxV);

unsigned get_random_uniform_unsigned(unsigned minV, unsigned maxV);

std::vector<double> sim_system(std::vector<double> state, std::vector<double> control_input, double timesteps);

//void print_tree(tree<Node> tree_input);