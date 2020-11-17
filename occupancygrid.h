//
// Created by etienne on 17.11.20.
//

#ifndef MCSAMPLING_OCCUPANCYGRID_H
#define MCSAMPLING_OCCUPANCYGRID_H


#include <cassert>
#include <vector>
#include <array>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <sstream>

#include "config.h"

struct OccupancyGrid{
public:
	OccupancyGrid();
	~OccupancyGrid() = default;

	int grid_[60][100];

	std::array<int, 60*100> get_grid_from_file();
	bool check_position(double x, double y);
};


#endif //MCSAMPLING_OCCUPANCYGRID_H