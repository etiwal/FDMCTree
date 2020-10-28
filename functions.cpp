//
// Created by etienne on 13.10.20.
//
#include <iostream>
#include <cstdlib>
#include <vector>


std::vector<std::vector<double>> sample_trajectory(int dimensions, int horizon){
    std::vector<std::vector<double>> i;
    i.resize(horizon, std::vector<double>(dimensions, 0));
    for (int dimension = 0; dimension < dimensions; ++dimension) {
        for (int step = 0; step < horizon; ++step) {
            if (step >= 1){
                i[step][dimension] = i[step-1][dimension] + ((double) random() / (RAND_MAX));
            }
            else {
                i[step][dimension] = ((double) random() / (RAND_MAX));
            }
        }
    }

    return i;
}

