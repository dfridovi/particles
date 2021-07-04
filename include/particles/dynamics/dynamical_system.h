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
// Junk header file so that git creates the right directoiry structure.
// TODO: erase with first real commit.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PARTICLES_DYNAMICS_DYNAMICAL_SYSTEM
#define PARTICLES_DYNAMICS_DYNAMICAL_SYSTEM

#include <particles/junk.h>
#include <particles/utils/types.h>
#include <iostream>

#include <vector>

namespace particles {

class DynamicalSystem {
 public:
  virtual ~DynamicalSystem() {}

  // Compute next state
  virtual VectorXf EvaluateNextState(int t, const VectorXf& x,
                                     const std::vector<VectorXf>& u) const = 0;

  // Compute current measurement

  virtual VectorXf EvaluateMeasurement(
      int t, const VectorXf& x, const std::vector<VectorXf>& u) const = 0;

 protected:
  DynamicalSystem(const int xdim, const int udim, const MatrixXf dynamics_covar,
                  const MatrixXf measurement_covar)
      : xdim_(xdim),
        udim_(udim),
        dynamics_covar_(dynamics_covar),
        measurement_covar_(measurement_covar) {}

  const int xdim_;
  const int udim_;
  const MatrixXf dynamics_covar_;
  const MatrixXf measurement_covar_;
}  // namespace particles

#endif
