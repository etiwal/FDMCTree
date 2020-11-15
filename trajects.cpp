//
// Created by etienne on 10.11.20.
//
#include "trajects.h"

// Traject
Traject::Traject(size_t sim_step, size_t rank): node_vec_(config::horizon){
	traject_length_ = config::horizon;
	sim_step_ = sim_step;
	rank_ = rank;
}

void Traject::append(const Node& node_object){
	node_vec_.push_back(node_object);
}
