//
// Created by etienne on 10.11.20.
//
#include "trajects.h"

// Traject
Traject::Traject(size_t sim_step, size_t rank){
	traject_length_ = config::horizon;
	sim_step_ = sim_step;
	rank_ = rank;
	node_vec_ = {};
}

void Traject::append(const Node& node_object){
	node_vec_.push_back(node_object);
}

void Traject::cut_first(){
	node_vec_.erase(node_vec_.begin());
}


//Trajects
void Trajects::append(size_t rank, size_t sim_step, const Traject& trajectory){
	trajectories_[sim_step].insert(std::make_pair(rank, trajectory));
};

Traject Trajects::get_best_prev_traject_cut(size_t sim_step) {
	size_t rank = 0;

	auto best_traject_prev = trajectories_[sim_step - 1].at(rank);
	best_traject_prev.cut_first();

	return best_traject_prev;
}
