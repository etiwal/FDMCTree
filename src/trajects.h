//
// Created by etienne on 10.11.20.
//

#ifndef MCSAMPLING_TRAJECTS_H
#define MCSAMPLING_TRAJECTS_H


#include <vector>
#include <unordered_map>

#include "node.h"

struct Traject{
public:
	Traject(size_t sim_step);
	~Traject() = default;

	void append(const Node& node_object);

	size_t sim_step_;
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

	Traject get_best_traject(size_t sim_step);

	Traject get_best_traject_cut(size_t sim_step);

	std::unordered_map<size_t, std::unordered_map<size_t, Traject>> trajectories_;
};


#endif //MCSAMPLING_TRAJECTS_H