#include <iostream>
#include "functions.h"
#include "fdmcts.h"
#include "tree.h"

//#include "matplotlibcpp.h"
//#include <cmath>
//
//namespace plt = matplotlibcpp;

int main(){
    int horizon = 5;
    int n_rollouts = 6;

	std::vector<double> initial_state = {0, 0, 0};

    Sys Robot(initial_state);

	for (int time = 0; time < 50; ++time) {
		// generate rollouts
		// init tree and root
		tree<Node> sampling_tree;

		// init all initial nodes as roots
		std::cout << std::endl;
		std::cout << "------------------------------------------------------------------------" << std::endl;
		std::cout << "Simulation time: " << time << std::endl;
		std::cout << "Robot is at position: " << Robot.get_state() << std::endl;
		std::cout << "Target is at position: " << 100 << std::endl;


		// create root of tree
		auto root = sampling_tree.insert(sampling_tree.begin(), Node(Robot.get_state(), 0, 0, 0));

		std::cout << std::endl;
		std::cout << "initializing nodes and an array with leaf iterator objects" << std::endl;
		std::vector<tree<Node>::iterator> leaf_handles(n_rollouts);

		for (int rollout = 0; rollout < n_rollouts; ++rollout) {
			auto init_node = sampling_tree.append_child(root, Node(Robot.get_state(), 0, 0, rollout));
			leaf_handles[rollout] = init_node;
		}

		for (int step = 0; step < horizon; ++step) {
			std::cout << "time-step: " << step << std::endl;

			std::cout << "Define Set of leaf handles which will be extended" << std::endl;
			std::vector<tree<Node>::iterator> leaf_handles_extending = {};

			// get the min cost of the leafs
			double min_cost = std::numeric_limits<double>::max();;
			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];
				if (active_rollout->cost_ < min_cost) {
					min_cost = active_rollout->cost_;
				}
			}

			std::cout << "current min cost is " << min_cost << std::endl;

			// construct a set of extandable leafs
			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];
				std::cout << "cost of active rollout is " << active_rollout->cost_ << std::endl;

				if (active_rollout->cost_ <= 1.2 * min_cost) {
					leaf_handles_extending.push_back(active_rollout);
				}
//			else {
//				std::cout << "rollout " << rollout << " has been cut" << std::endl;
//			};
			}

			std::cout << "currently we have n extendable leafs: " << leaf_handles_extending.size() << std::endl;

			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];
				// check if active_rollout is in extendable vector ...
				if (std::find(leaf_handles_extending.begin(), leaf_handles_extending.end(), active_rollout) !=
					leaf_handles_extending.end()) {
					std::cout << "active_rollout will be directly extended" << std::endl;
					// based on the state and the control input the system is propagated
					active_rollout->set_control_input(active_rollout->state_, active_rollout->expert_type_);

					double next_state = sim_system(active_rollout->state_, active_rollout->control_input_, 1);
					leaf_handles[rollout] = sampling_tree.append_child(active_rollout,
																	   Node(next_state, step, active_rollout->expert_type_,
																			rollout));
				} else {
					unsigned random_unsigned = get_random_uniform_unsigned(0, leaf_handles_extending.size()-1);

					std::cout << "active_rollout will be extended on leaf " << random_unsigned << std::endl;
					auto extending_leaf = leaf_handles_extending[random_unsigned];

					active_rollout->set_control_input(extending_leaf->state_, active_rollout->expert_type_);

					// based on the state and the control input the system is propagated
					double next_state = sim_system(extending_leaf->state_, active_rollout->control_input_, 1);
					leaf_handles[rollout] = sampling_tree.append_child(extending_leaf,
																	   Node(next_state, step, active_rollout->expert_type_,
																			rollout));

				}

			}
		}


		// print tree
		std::cout << std::endl;

		tree<Node>::iterator start_node = sampling_tree.begin(sampling_tree.begin());
		tree<Node>::iterator end_node = sampling_tree.end(sampling_tree.end());

		while (start_node != end_node) {
			int node_depth = sampling_tree.depth(start_node);

			for (int i = 0; i < node_depth-1; ++i) {
				if (i==node_depth-2){
					std::cout << "+- ";
				} else {
					std::cout << "|  ";
				}
			}

			std::cout << "state: " << std::round(start_node->state_) << ", cost: " << std::round(start_node->cost_) << std::endl;

			start_node++;
		}


		// get the min control input of the generated child with the lowest simulated cost
		double min_cost = std::numeric_limits<double>::max();
		auto best_leaf_handle = leaf_handles[0];
		for (int rollout = 0; rollout < n_rollouts; ++rollout) {
			auto active_rollout = leaf_handles[rollout];
			if (active_rollout->cost_ < min_cost) {
				min_cost = active_rollout->cost_;
				best_leaf_handle = leaf_handles[rollout];
			}
		}

		std::vector<int> best_rollout_path(sampling_tree.path_from_iterator(best_leaf_handle, sampling_tree.begin()));
		auto best_root_handle = sampling_tree.iterator_from_path({0, best_rollout_path[1]}, sampling_tree.begin());

		std::cout << std::endl;
		std::cout << "Winning rollout has final state cost: " << best_leaf_handle->cost_ << std::endl;
		std::cout << "Control input " << best_root_handle->control_input_ << " is applied to Robot" << std::endl;
		Robot.apply_control_input(best_root_handle->control_input_);
		std::cout << "Robot moved to position " << Robot.get_state() << std::endl;
	}



	return 0;
}