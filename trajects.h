//
// Created by etienne on 10.11.20.
//

#ifndef MCSAMPLING_TRAJECTS_H
#define MCSAMPLING_TRAJECTS_H

#endif //MCSAMPLING_TRAJECTS_H

#include <vector>
#include "fdmcts.h"

struct Traject{
public:
	Traject(size_t sim_step, size_t rank);
	~Traject() = default;

	void append(const Node& node_object);

	size_t horizon_step_;
	size_t sim_step_;
	size_t rank_;


private:
	size_t traject_length_;
	std::vector<Node> node_vec_;
};

struct Trajects{
public:
	Trajects();
	~Trajects() = default;

	Traject get_best_prev_traject();

private:
	//Traject
	//here comes some trajectory storage (to store the k best traj from the all prev iterations)
};