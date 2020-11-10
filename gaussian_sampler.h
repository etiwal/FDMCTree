//
// Created by etienne on 09.11.20.
//

#ifndef MCSAMPLING_GAUSSIAN_SAMPLER_H
#define MCSAMPLING_GAUSSIAN_SAMPLER_H

#endif //MCSAMPLING_GAUSSIAN_SAMPLER_H

/*!
 * @file     gaussian_sampler.h
 * @author   Giuseppe Rizzi
 * @date     21.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <memory>
#include "mulitvariate_normal_eigen.h"
#include <iostream>

class GaussianSampler {
public:
	using sampler_ptr = std::shared_ptr<GaussianSampler>;

	GaussianSampler() = default;
	explicit GaussianSampler(size_t n) : n_(n), solver_(n) {
		mean_ = Eigen::VectorXd::Zero(n_);
		sigma_ = Eigen::MatrixXd::Identity(n_, n_);
		sigma_inv_ = Eigen::MatrixXd::Identity(n_, n_);
		dist_ = std::make_unique<multivariate_normal>(mean_, sigma_);
	};

	~GaussianSampler() = default;

	void set_covariance(const double s) {
		assert(s != 0);
		sigma_ = Eigen::MatrixXd::Identity(n_, n_) * s;
		sigma_inv_ = Eigen::MatrixXd::Identity(n_, n_) * 1. / s;
		dist_->set_covariance(sigma_);
	}

	void set_covariance(Eigen::MatrixXd &v) {
		assert(1==0);
		assert(v.rows() == v.cols() && v.rows() == n_);
		sigma_ = v;
		sigma_inv_ = stable_inverse(v);
		dist_->set_covariance(sigma_);
	}

	template<typename T>
	void set_covariance(const T &s) {
		assert(s.size() == n_);
		for (size_t i = 0; i < n_; i++) {
			sigma_(i, i) = s[i];
			sigma_inv_(i, i) = 1. / s[i];
			dist_ = std::make_unique<multivariate_normal>(sigma_);
		}
		dist_->set_covariance(sigma_);
	}

	template<typename T>
	void set_mean(const T &s) {
		assert(s.size() == n_);
		for (size_t i = 0; i < n_; i++) {
			mean_(i) = s[i];
		}
		dist_->set_mean(mean_);
	}

	Eigen::MatrixXd get_sample() {
		auto sample = (*dist_)();
		return sample;
	}

	void combine_dist_mult(const GaussianSampler& dist_2) {
		// based on http://compbio.fmph.uniba.sk/vyuka/ml/old/2008/handouts/matrix-cookbook.pdf
		std::vector<double> sigma_new_vector(n_);
		Eigen::VectorXd mean_new(n_);
		Eigen::MatrixXd sigma_new(n_, n_);
		Eigen::MatrixXd sigma_inv_new(n_, n_);

		auto mean_2 = dist_2.mean_;
		auto sigma_2 = dist_2.sigma_;
		auto sigma_inv_2 = dist_2.sigma_inv_;

		// calc new sigma
		sigma_inv_new = sigma_inv_ + sigma_inv_2;
		for (int i = 0; i < n_; ++i) {
			sigma_new(i,i) = 1. / sigma_inv_new(i,i);
		}

		for (int i = 0; i < n_ ; ++i) {
			sigma_new_vector[i] = sigma_new(i,i);
		}

		// calc new mean
		mean_new = (sigma_new * sigma_inv_ * mean_) + (sigma_new * sigma_inv_2 * mean_2);

		this->set_covariance(sigma_new_vector);
		this->set_mean(mean_new);
	}

	void combine_dist_KL(const GaussianSampler& dist_new) {
		perror ("this mode is not yet supported");
	}

	Eigen::MatrixXd stable_inverse(const Eigen::MatrixXd &A) {
		solver_.compute(A, Eigen::ComputeEigenvectors);
		if (solver_.info() != Eigen::Success) {
			std::cout << "Something went wrong. Sigma: " << "\n" << A << std::endl;
			throw std::runtime_error("Eigenvalue decomposition failed");
		}

		Eigen::VectorXd sigma = solver_.eigenvalues().cwiseSqrt();
		Eigen::VectorXd sigma_inverse = sigma;
		double sigma_min = sigma(0);
		std::cout << "sigma" << sigma.transpose() << std::endl;
		std::cout << "sigma min" << sigma_min << std::endl;

		// TODO this fails to stabilize the inverse
		for (int k = 0; k < A.rows(); ++k) {
			if (sigma(k) < 1e-6)
				sigma_inverse(k) = 1.0 / (sigma(k) + 1e-6);
			else
				sigma_inverse(k) = 1.0 / sigma(k);
		}

		return solver_.eigenvectors() * sigma_inverse.asDiagonal() * solver_.eigenvectors().transpose();
	}

	inline Eigen::MatrixXd const &sigma() const { return sigma_; }

	inline Eigen::MatrixXd const &sigma_inv() const { return sigma_inv_; }

private:
	size_t n_;
	Eigen::MatrixXd mean_;
	Eigen::MatrixXd sigma_;
	Eigen::MatrixXd sigma_inv_;
	std::shared_ptr<multivariate_normal> dist_;

	Eigen::SelfAdjointEigenSolver <Eigen::MatrixXd> solver_;
};