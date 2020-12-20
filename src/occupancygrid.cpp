//
// Created by etienne on 17.11.20.
//

#include "occupancygrid.h"

OccupancyGrid::OccupancyGrid() {
	auto grid_one_d_array = get_grid_from_file();

	for (int row = 0; row < config::grid_height; ++row) {
		for (int column = 0; column < config::grid_width ; ++column) {
			grid_[row][column] = grid_one_d_array[row*config::grid_width+column];
		}
	}
}

bool OccupancyGrid::check_position(double x, double y) {
	int x_min = std::floor(x);
	int y_min = std::floor(y);

	bool occupied = false;
	if (x<0 | x>config::grid_width | y < 0 | y > config::grid_height){
		occupied = true;
	} else if (grid_[config::grid_height-1-y_min][x_min]!=0){
		occupied = true;
	} else {
		occupied = false;
	}

	return occupied;
}

std::array<int, 60*100> OccupancyGrid::get_grid_from_file() {
	std::array<int, 60*100> one_d_array{};

	// File pointer
	std::fstream fin;

	// Open an existing file
	fin.open(config::grid_path, std::ios::in);
	assert(fin.is_open());

	// Read the Data from the file
	// as String Vector
	std::vector<int> row;
	std::string line, word, temp;
	int i = 0;

	while (!fin.eof()) {

		row.clear();

		// read an entire row and
		// store it in a string variable 'line'
		getline(fin, line);

		// used for breaking words
		std::stringstream s(line);

		// read every column data of a row and
		// store it in a string variable, 'word'
//		std::cout << "now" << std::endl;
		int j = 0;
		while (getline(s, word, ',')) {
			row.push_back(stoi(word));
			one_d_array[i*config::grid_width+j] = stoi(word);
			j++;
		}
		i++;
	}
	return 	one_d_array;
}