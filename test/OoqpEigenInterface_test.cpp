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
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"
#include <Eigen/Core>
#include <Eigen/SparseCore>

using namespace Eigen;
using namespace ooqpei;
using namespace ooqpei::eigen;
using namespace std;

TEST(OOQPEITest, LeastSquares)
{
  Vector2d solution(10.0, 8.0);

  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(2, 2);
  Q.insert(0, 0) = 1.0;
  Q.insert(0, 1) = -1.0;
  Q.insert(1, 0) = -1.0;
  Q.insert(1, 1) = 2.0;
  VectorXd c(2);
  c << -2.0, -6.0;
  VectorXd x;

  ooqpei::OoqpEigenInterface::solve(Q, c, x);

  expectNear(x, solution, 1e-8, OOQPEI_SOURCE_FILE_POS);
}

TEST(OOQPEITest, InequalityConstraints)
{
  Vector2d solution(2.0 / 3.0, 1.0 + 1.0 / 3.0);

  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(2, 2);
  Q.insert(0, 0) = 1.0;
  Q.insert(0, 1) = -1.0;
  Q.insert(1, 0) = -1.0;
  Q.insert(1, 1) = 2.0;
  VectorXd c(2);
  c << -2.0, -6.0;
  SparseMatrix<double, Eigen::RowMajor> A;
  VectorXd b;
  SparseMatrix<double, Eigen::RowMajor> C;
  C.resize(3, 2);
  C.insert(0, 0) = 1.0;
  C.insert(0, 1) = 1.0;
  C.insert(1, 0) = -1.0;
  C.insert(1, 1) = 2.0;
  C.insert(2, 0) = 2.0;
  C.insert(2, 1) = 1.0;
  VectorXd d(3);
  d << -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max();
  VectorXd f(3);
  f << 2.0, 2.0, 3.0;
  VectorXd l(2);
  l << 0.0, 0.0;
  VectorXd u(2);
  u << 1000.0, 1000.0;
  VectorXd x;

  OoqpEigenInterface::solve(Q, c, A, b, C, d, f, l, u, x);

  expectNear(x, solution, 1e-8, OOQPEI_SOURCE_FILE_POS);
}

TEST(OOQPEITest, LinearProgrammingNoConstraints)
{
  Vector3d solution(0.0, 0.0, 0.0);

  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(3, 3);
  VectorXd c(3);
  c << 5.0, 4.0, 6.0;
  SparseMatrix<double, Eigen::RowMajor> A;
  VectorXd b;
  VectorXd l = VectorXd::Zero(3);
  VectorXd u = Vector3d::Constant(std::numeric_limits<double>::max());
  VectorXd x;

  OoqpEigenInterface::solve(Q, c, A, b, l, u, x);

  expectNear(x, solution, 1e-8, OOQPEI_SOURCE_FILE_POS);
}

TEST(OOQPEITest, LinearProgrammingWithInequalityConstraints)
{
  Vector3d solution(0.0, 15.0, 3.0);

  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(3, 3);
  VectorXd c(3);
  c << -5.0, -4.0, -6.0;
  SparseMatrix<double, Eigen::RowMajor> A;
  VectorXd b;
  SparseMatrix<double, Eigen::RowMajor> C;
  C.resize(3, 3);
  C.insert(0, 0) =  1.0;
  C.insert(0, 1) = -1.0;
  C.insert(0, 2) =  1.0;
  C.insert(1, 0) =  3.0;
  C.insert(1, 1) =  2.0;
  C.insert(1, 2) =  4.0;
  C.insert(2, 0) =  3.0;
  C.insert(2, 1) =  2.0;
  C.insert(2, 2) =  0.0;
  VectorXd d(3);
  d = Vector3d::Constant(-std::numeric_limits<double>::max());
  VectorXd f(3);
  f << 20.0, 42.0, 30.0;
  VectorXd l = VectorXd::Zero(3);
  VectorXd u(3);
  u = Vector3d::Constant(std::numeric_limits<double>::max());
  VectorXd x;

  OoqpEigenInterface::solve(Q, c, A, b, C, d, f, l, u, x);

  expectNear(x, solution, 1e-8, OOQPEI_SOURCE_FILE_POS);
}

TEST(OOQPEITest, NaN)
{
  SparseMatrix<double, Eigen::RowMajor> Q;
  Q.resize(2, 2);
  Q.insert(0, 0) = NAN;
  Q.insert(0, 1) = NAN;
  Q.insert(1, 0) = NAN;
  Q.insert(1, 1) = NAN;
  VectorXd c(2);
  c << NAN, NAN;
  VectorXd x;

  EXPECT_TRUE(ooqpei::OoqpEigenInterface::solve(Q, c, x));
}
