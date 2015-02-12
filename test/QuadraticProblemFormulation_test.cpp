/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Péter Fankhauser, Christian Gehring, Stelian Coros
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
* @file    QuadraticProblemFormulation_test.cpp
* @author  Péter Fankhauser
* @date    Aug 16, 2013
*/

#include <gtest/gtest.h>
#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"
#include <Eigen/Core>

using namespace Eigen;
using namespace ooqpei;
using namespace ooqpei::eigen;
using namespace std;

TEST(OOQPEITest, QuadraticProblemFormulation)
{
  Vector2d solution(1.030779872038734, -3.143697043057237);

  Eigen::SparseMatrix<double, Eigen::RowMajor> A, C, D;
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S, W;
  Eigen::VectorXd b, c, d, f, x;

  A.resize(3, 2);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = -1.0;
  A.insert(1, 0) = -4.0;
  A.insert(1, 1) = -3.0;
  A.insert(2, 0) = 2.0;
  A.insert(2, 1) = 0.0;
  S = Vector3d(3.0, 2.0, 0.69).asDiagonal();
  W = Vector2d(0.5, 0.2).asDiagonal();
  b.resize(3);
  b << 3.0, 6.0, 9.0;

  QuadraticProblemFormulation::solve(A, S, b, W, C, c, D, d, f, x);

  expectNear(x, solution, 1e-10, OOQPEI_SOURCE_FILE_POS);
}
