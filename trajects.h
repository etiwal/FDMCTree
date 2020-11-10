//
// Created by etienne on 10.11.20.
//

#ifndef MCSAMPLING_TRAJECTS_H
#define MCSAMPLING_TRAJECTS_H

#endif //MCSAMPLING_TRAJECTS_H

#include <vector>
#include "fdmcts.h"

struct Trajects{
public:
	Trajects();
	~Trajects() = default;

	std::vector<Node> get_best_traj_prev();

private:
	//Traject
	//here comes some trajectory storage (to store the k best traj from the all prev iterations)
};

struct Traject{
public:
	Traject();
	~Traject() = default;

private:
	//here comes some trajectory storage (to store the k best traj from the all prev iterations)
};