//
// Created by etienne on 10.11.20.
//

#ifndef MCSAMPLING_TRAJECTS_H
#define MCSAMPLING_TRAJECTS_H

#endif //MCSAMPLING_TRAJECTS_H

#include <vector>
#include <unordered_map>

#include "fdmcts.h"

struct Traject{
public:
	Traject(size_t sim_step, size_t rank);
	~Traject() = default;

	void append(const Node& node_object);

	size_t sim_step_;
	size_t rank_;
	std::vector<Node> node_vec_;

	void cut_first();

private:
	size_t traject_length_;

};

struct Trajects{
public:
	Trajects() = default;
	~Trajects() = default;

	void append(size_t rank, size_t sim_step, const Traject& trajectory);

	Traject get_best_prev_traject_cut(size_t sim_step);

private:
	std::unordered_map<size_t, std::unordered_map<size_t, Traject>> trajectories_;
};