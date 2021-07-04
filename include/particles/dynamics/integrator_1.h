/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Junk header file so that git creates the right directory structure.
// TODO: erase with first real commit.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PARTICLES_DYNAMICS_INTEGRATOR_1
#define PARTICLES_DYNAMICS_INTEGRATOR_1

#include <particles/junk.h>
#include <particles/utils/types.h>
#include <cmath>
#include <iostream>
#include <random>

#include <vector>

#include <particles/dynamics/linear_dynamical_system.h>

// Dynamics, Measurement maps:
// x(t+1) = A*x(t) + B*u(t) + w(t);
// y(t) = C*x(t) + v(t);
// A = [0, 1; 0, 0]; B = [0; 1]; C = [1, 0; 0, 1];
// mu_W = [0; 0], Sigma_W = [0.1, 0; 0; 0.1]; mu_V = [0; 0], Sigma_V = [0.2, 0;
// 0, 0.2];

// void HelloWorld() { std::cout << "delete me!" << std::endl; }

}  // namespace particles

namespace particles {

class Integrator1 : public LinearDynamicalSystem {
 public:
  ~Integrator1() {}

  Integrator1(const MatrixXf& A, const MatrixXf& B)
      : LinearDynamicalSystem(kNumXDims, kNumUDims) {}

  inline VectorXf Integrator1::EvaluateNextState(
      int t, const VectorXf& x, const MatrixXf& B,
      const std::vector<VectorXf>& u) const {
    // To do: Add noise
    // Use Eigen::EigenMultivariateNormal

    static const VectorXf mean_zero_dym = MatrixXf::Zero(xdim_);

    Eigen::EigenMultivariateNormal<float> multivariate_normal_generator(
        mean_zero_dym, dynamics_covar_);

    MatrixXf w;
    w << multivariate_normal_generator.samples(1);

    return A * x + B * u + w
  }

  inline VectorXf Integrator1::EvaluateMeasurement(int t, const VectorXf& x,
                                                   const MatrixXf& C) const {
    // To do: Add noise
    // Use Eigen::EigenMultivariateNormal

    static const VectorXf mean_zero_meas = MatrixXf::Zero(xdim_);

    Eigen::EigenMultivariateNormal<float> multivariate_normal_generator(
        mean_zero_meas, measurement_covar_);

    MatrixXf v;
    v << multivariate_normal_generator.samples(1);

    return C * x + v
  }

  // To create: constructor, etc.
}

}  // namespace particles

#endif
