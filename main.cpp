#include <iostream>
#include <string>
#include <boost/format.hpp>

#include "functions.h"
#include "tree.h"
#include "logger.h"
#include "sys_sim.h"
#include "trajects.h"

//#include "matplotlibcpp.h"
//#include <cmath>
//
//namespace plt = matplotlibcpp;

int main(){
	// Init Logging
	Logger config_log { "config_log.txt" };
	config_log.write("sim_time", "rollouts", "horizon", "obstacle_cost", "obstacle_rad_0", "obstacle_rad_1", "obstacle_rad_2","obstacle_rad_3", "obstacle_pos_0_x","obstacle_pos_0_y", "obstacle_pos_1_x","obstacle_pos_1_y", "obstacle_pos_2_x","obstacle_pos_2_y", "obstacle_pos_3_x","obstacle_pos_3_y", "target_state_0", "target_state_1");
	config_log.write_endl();
	config_log.write(config::sim_time, config::rollouts, config::horizon, config::obstacle_cost, config::obstacle_rad[0],config::obstacle_rad[1],config::obstacle_rad[2],config::obstacle_rad[3], config::obstacle_pos[0],config::obstacle_pos[1],config::obstacle_pos[2],config::obstacle_pos[3],config::obstacle_pos[4],config::obstacle_pos[5],config::obstacle_pos[6],config::obstacle_pos[7],config::target_state[0],config::target_state[1]);
	config_log.write_endl();

	Logger sim_log { "sim_log.txt" };
	sim_log.write("simstep", "state_0", "state_1", "state_2", "state_3", "state_4", "state_5");
	sim_log.write_endl();

	Logger sampling_log { "sampling_log.txt" };
	sampling_log.write("node_id", "simstep", "step","rollout","state_0", "state_1", "state_2", "state_3", "state_4", "state_5", "cost_cum", "path_to_leaf");
	sampling_log.write_endl();

	//YAML::Node ConfigNode = YAML::LoadFile("./config.yaml");

    int horizon = config::horizon;//ConfigNode["solver"]["horizon"].as<int>();
    int n_rollouts = config::rollouts;//ConfigNode["solver"]["rollouts"].as<int>();

	std::vector<double> initial_state = config::initial_state;

    Sys Robot(initial_state);
    Trajects trajectories;

    extern Expert Expert_Instance();

	for (int time = 0; time < config::sim_time; ++time) {
		// generate rollouts
		// init tree and root

		if (time==0){
			sim_log.write(time, Robot.get_state()[0], Robot.get_state()[1], Robot.get_state()[2], Robot.get_state()[3], Robot.get_state()[4], Robot.get_state()[5]);
			sim_log.write_endl();
		}

		tree<Node> sampling_tree;

		// init all initial nodes as roots
		std::cout << std::endl;
		std::cout << "------------------------------------------------------------------------" << std::endl;
		std::cout << "Simulation time: " << time << std::endl;
		std::cout << "Robot is at position: (" << Robot.get_state()[0] << ", " << Robot.get_state()[1] << ")" << std::endl;
		std::cout << "Target is at position: (" << config::target_state[0] << ", " << config::target_state[1] << ")" << std::endl;


		// create root of tree
		GaussianSampler sampler_root(2);
		auto root = sampling_tree.insert(sampling_tree.begin(), Node(0, {}, Robot.get_state(), 0, 0, 0, sampler_root));

		std::cout << std::endl;
		debug_print(2, boost::format("initializing nodes and an array with leaf iterator objects"));
		std::vector<tree<Node>::iterator> leaf_handles(n_rollouts);

		for (int rollout = 0; rollout < n_rollouts; ++rollout) {
			if (config::use_last_best == true && rollout == 0 && time != 0){
				auto best_prev_traj_cut = trajectories.get_best_prev_traject_cut(time);
				auto best_prev_traj_node_current = best_prev_traj_cut.node_vec_[0];

				auto node_id = get_unique_node_id(0,0,rollout,true);
				auto init_node = sampling_tree.append_child(root, Node(node_id, root->node_id_path_, Robot.get_state(), 0, rollout,0, best_prev_traj_node_current.sampler_));
				leaf_handles[rollout] = init_node;
			} else {
				GaussianSampler sampler_init(2);

				auto node_id = get_unique_node_id(0,0,rollout,true);
				auto init_node = sampling_tree.append_child(root, Node(node_id, root->node_id_path_, Robot.get_state(), 0, rollout,0, sampler_init));
				leaf_handles[rollout] = init_node;
			}
		}

		for (int step = 0; step < horizon; ++step) {
			debug_print(2, boost::format("time-step: %1%") % step);
			debug_print(2, boost::format("Define Set of leaf handles which will be extended"));
			debug_print(2, boost::format(""));

			std::vector<tree<Node>::iterator> leaf_handles_extending = {};

			// get the min cost of the leafs
			double min_cost = std::numeric_limits<double>::max();;
			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];
				if (active_rollout->cost_cum_ < min_cost) {
					min_cost = active_rollout->cost_cum_;
				}
			}

//			std::cout << "current min cost is " << min_cost << std::endl;

			// construct a set of extandable leafs
			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];
//				std::cout << "cost of active rollout is " << active_rollout->cost_ << std::endl;
				if (config::use_last_best == true && rollout == 0 && time != 0 && step < horizon-1){
					leaf_handles_extending.push_back(active_rollout);
				}
				// TODO: could be set to 2 times the cost of the value from last iterations best trajectory (implementation in paper)
				else if(active_rollout->cost_cum_ <= min_cost + (config::pruning_threshold * min_cost)) {
					leaf_handles_extending.push_back(active_rollout);
				}
//			else {
//				std::cout << "rollout " << rollout << " has been cut" << std::endl;
//			};
			}

			debug_print(2, boost::format("currently we have n extendable leafs: %1%") % leaf_handles_extending.size());

			for (int rollout = 0; rollout < n_rollouts; ++rollout) {
				auto active_rollout = leaf_handles[rollout];

				if (config::use_last_best == true && rollout == 0 && time != 0 && step < horizon-1){
					debug_print(2, boost::format("previous best will be directly extended"));

					auto best_prev_traj_cut = trajectories.get_best_prev_traject_cut(time);
					auto best_prev_traj_node_current = best_prev_traj_cut.node_vec_[step+1];

					active_rollout->set_control_input(best_prev_traj_node_current.control_input_);

					std::vector<double> next_state = sim_system(active_rollout->state_, active_rollout->control_input_, 1);

					// TODO: Implement Sampler and other important parameters
					auto node_id = get_unique_node_id(time,step,rollout,false);
					leaf_handles[rollout] = sampling_tree.append_child(active_rollout,
																	   Node(node_id, active_rollout->node_id_path_,next_state, step, rollout,active_rollout->cost_cum_, best_prev_traj_node_current.sampler_));
				}
				// check if active_rollout is in extendable vector ...
				else if (std::find(leaf_handles_extending.begin(), leaf_handles_extending.end(), active_rollout) !=
					leaf_handles_extending.end()) {
					debug_print(2, boost::format("active_rollout will be directly extended"));
					// based on the state and the control input the system is propagated

//					active_rollout->sample_control_input(active_rollout->state_, active_rollout->expert_type_);
					std::vector<double> control_input(2);
					auto sampled_control_input = active_rollout->sampler_.get_sample();
					control_input[0] = sampled_control_input(0,0);
					control_input[1] = sampled_control_input(1,0);

					active_rollout->set_control_input(control_input);

					std::vector<double> next_state = sim_system(active_rollout->state_, active_rollout->control_input_, 1);

					auto node_id = get_unique_node_id(time,step,rollout,false);
					leaf_handles[rollout] = sampling_tree.append_child(active_rollout,
																	   Node(node_id, active_rollout->node_id_path_, next_state, step, rollout,active_rollout->cost_cum_, active_rollout->sampler_));
				} else {
					unsigned random_unsigned = get_random_uniform_unsigned(0, leaf_handles_extending.size()-1);

					debug_print(2, boost::format("active_rollout will be extended on leaf %1%") % random_unsigned);
					auto extending_leaf = leaf_handles_extending[random_unsigned];

//					active_rollout->sample_control_input(extending_leaf->state_, active_rollout->expert_type_);
					std::vector<double> control_input(2);
					auto sampled_control_input = active_rollout->sampler_.get_sample();
					control_input[0] = sampled_control_input(0,0);
					control_input[1] = sampled_control_input(1,0);

					active_rollout->set_control_input(control_input);


					// based on the state and the control input the system is propagated
					std::vector<double> next_state = sim_system(extending_leaf->state_, active_rollout->control_input_, 1);

					auto node_id = get_unique_node_id(time,step,rollout,false);
					leaf_handles[rollout] = sampling_tree.append_child(extending_leaf,
																	   Node(node_id, extending_leaf->node_id_path_, next_state, step, rollout, extending_leaf->cost_cum_, extending_leaf->sampler_));

				}

			}
			if (config::log_sampling==true){
				for (int i = 0; i < leaf_handles.size(); ++i) {
					std::string path_to_leaf_str;
					std::vector<size_t> node_id_path = leaf_handles[i]->node_id_path_;
					for (int path_step = 0; path_step < node_id_path.size(); ++path_step) {
						path_to_leaf_str.append(std::to_string(node_id_path[path_step]));
						if (path_step != node_id_path.size()-1){
							std::string seperator = ";";
							path_to_leaf_str.append(seperator);
						}
					}
//					// declaring character array : p
//					char path_to_leaf_char[path_to_leaf_str.length()+1];
//					for (i = 0; i < sizeof(path_to_leaf_char); i++) {path_to_leaf_char[i] = path_to_leaf_str[i];}

					sampling_log.write(leaf_handles[i]->node_id_, time, step,i,leaf_handles[i]->state_[0],leaf_handles[i]->state_[1],leaf_handles[i]->state_[2],leaf_handles[i]->state_[3],leaf_handles[i]->state_[4],leaf_handles[i]->state_[5], leaf_handles[i]->cost_cum_, path_to_leaf_str);
					sampling_log.write_endl();
				}
			}

		}

		tree<Node>::iterator start_node = sampling_tree.begin();
		tree<Node>::iterator end_node = sampling_tree.end();

		if (config::debug_level >= 1) {
			while (start_node != end_node) {
				int node_depth = sampling_tree.depth(start_node);

				for (int i = 0; i < node_depth; ++i) {
					if (i == node_depth - 1) {
						std::cout << "+- ";
					} else {
						std::cout << "|  ";
					}
				}

				std::cout << "state: (" << std::round(start_node->state_[0]) << ", "
						  << std::round(start_node->state_[1]) << "), cost: " << std::round(start_node->cost_) << "/"
						  << std::round(start_node->cost_cum_) << ", expert type: "
						  << std::round(start_node->expert_type_) << std::endl;

				start_node++;
			}
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

		Traject trajectory(time, 0);
		for (int i = 0; i < best_rollout_path.size(); ++i) {
			std::vector<int> sub_path_rollout;
			for (int j = 0; j <= i; ++j) {
				sub_path_rollout.push_back(best_rollout_path[j]);
			}
			auto current_handle = sampling_tree.iterator_from_path(sub_path_rollout, sampling_tree.begin());
			trajectory.append(*current_handle);
		}

		trajectories.append(0,time,trajectory);

		std::cout << std::endl;
		std::cout << "Winning rollout has final state cost: " << best_leaf_handle->cost_ << std::endl;
		std::cout << "Control input (" << best_root_handle->control_input_[0] << ", " << best_root_handle->control_input_[1] <<") is applied to Robot" << std::endl;
		Robot.apply_control_input(best_root_handle->control_input_, 1);
		std::cout << "Robot moved to position (" << Robot.get_state()[0] << ", " << Robot.get_state()[1] << "), Target was: (" << config::target_state[0] << ", " << config::target_state[1] << ")" << std::endl;

		sim_log.write(time+1, Robot.get_state()[0], Robot.get_state()[1], Robot.get_state()[2], Robot.get_state()[3], Robot.get_state()[4], Robot.get_state()[5]);
		sim_log.write_endl();
	}



	return 0;
}