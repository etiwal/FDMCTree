//
// Created by etienne on 09.11.20.
//

#ifndef MCSAMPLING_MULITVARIATE_NORMAL_EIGEN_H
#define MCSAMPLING_MULITVARIATE_NORMAL_EIGEN_H



/*!
 * @file     multivariate_normal_eigen.h
 * @author   Giuseppe Rizzi
 * @date     21.07.2020
 * @version  1.0
 * @brief    description
 */

#include <random>
#include <Eigen/Dense>


// Implementation from the following post
// https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c

/**
 * @brief Utitlity class to perform sampling from a multivariate normal distribution
 */
struct multivariate_normal {
	multivariate_normal(Eigen::MatrixXd const& covar)
			: multivariate_normal(Eigen::VectorXd::Zero(covar.rows()), covar)
	{}

	multivariate_normal(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
			: mean(mean)
	{
		set_covariance(covar);
	}

	void set_mean(Eigen::VectorXd const& m){ mean = m; }

	void set_covariance(Eigen::MatrixXd const& cov){
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(cov);
		transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	}


	Eigen::VectorXd mean;
	Eigen::MatrixXd transform;

	Eigen::VectorXd operator()() const {
		static std::mt19937 gen{ std::random_device{}() };
		static std::normal_distribution<double> dist;

		return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
	}

//	void mult_dist(Eigen::VectorXd const& mean_, Eigen::MatrixXd const& sigma_, Eigen::MatrixXd const& sigma_inv_, Eigen::VectorXd const& mean_2, Eigen::MatrixXd const& sigma_2, Eigen::MatrixXd const& sigma_inv_2) {
//		//initialize new parameters
//		Eigen::VectorXd mean_new;
//		Eigen::MatrixXd sigma_new;
//		Eigen::MatrixXd sigma_inv_new;
//
//		// calc value of new sigma
//		sigma_inv_new = (sigma_inv_ + sigma_inv_2)
//		sigma_new =
//
//		mean_new =
//		mean = mean_new;
//
//		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(cov);
//		transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
//
//		std::cout << "this mode is not yet supported!" << std::endl;
//	}
//
//	void KL_dist() {
//		std::cout << "this mode is not yet supported!" << std::endl;
//	}
};

#endif //MCSAMPLING_MULITVARIATE_NORMAL_EIGEN_H
