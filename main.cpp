#include <iostream>
#include "functions.h"
#include "fdmcts.h"
#include "tree.h"
#include "tree_util.h"

//#include "matplotlibcpp.h"
//#include <cmath>
//
//namespace plt = matplotlibcpp;

int main(){

    int state_dimensions = 2;
    //int control_dimensions = 2;
    int horizon = 4;
    int n_rollouts = 2;

    double init_state = 1;

    // init tree and root
	tree<Node> sampling_tree;
	tree<Node>::iterator root = sampling_tree.insert(sampling_tree.begin(),Node(0,0,0));


	// init all initial nodes
	for (int rollout = 0; rollout < n_rollouts; ++rollout) {
		Node current_node = Node(init_state, 0, rollout);
		auto some_init_node = sampling_tree.append_child(root,current_node);
	}

	for (int i = 0; i < horizon; ++i) {
		std::cout << "time-step: " << i << std::endl;
		tree<Node>::iterator root_of_tree = sampling_tree.begin();
		root_of_tree++;
		tree<Node>::fixed_depth_iterator current_node = sampling_tree.begin_fixed(root_of_tree, i);
		tree<Node>::fixed_depth_iterator last_current_node = sampling_tree.end_fixed(root_of_tree, i);

		while (current_node != last_current_node){
			// sample optimal control input based on current nodes state
			//kptree::print_tree_bracketed(sampling_tree);
			tree<Node>::iterator begin = sampling_tree.begin();
			tree<Node>::iterator end = sampling_tree.end();


			while (begin != end)
			{
				std::cout << begin->cost_ << std::endl;
				begin++;
			}
			std::cout << "" << std::endl;




//			std::cout << current_node->control_input_ << std::endl;
			current_node->set_control_input(2);
//			std::cout << current_node->control_input_ << std::endl;
			// For child
			// calc one step dynamics
			// calc cost
			// create new child
			sampling_tree.append_child(current_node, Node(i, i, i));

			current_node++;
		}


	}




//    std::vector<std::vector<std::vector<double>>> trajectories;
//
//	for (int i = 0; i < n_rollouts; ++i) {
//		trajectories[i] = sample_trajectory(state_dimensions, horizon);
//	}

//	tree<Node>::iterator root = test_tree.insert(test_tree.begin(),Node(0,0,1,1));
//	tree<Node>::iterator first = test_tree.append_child(root,Node(10,1,1,1));
//	tree<Node>::iterator second = test_tree.append_child(root,Node(20,1,1,1));
//
//	for (int i = 0; i < 4; ++i) {
//		test_tree.append_child(second, Node(i,0,0,0));
//	}
//

//	tree<Node>::leaf_iterator begin = sampling_tree.begin_leaf();
//	tree<Node>::leaf_iterator end = sampling_tree.end_leaf();
//
//
//	while (begin != end)
//	{
//		std::cout << begin->cost_ << std::endl;
//		begin++;
//	}

    return 0;
}
