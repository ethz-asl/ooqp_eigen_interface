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
* @file    QuadraticProblemFormulation.cpp
* @author  Péter Fankhauser, Christian Gehring
* @date    Aug 16, 2013
*/

#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/ooqpei_assert_macros.hpp"
#include <stdexcept>
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"

using namespace Eigen;
using namespace std;

namespace ooqpei {

bool QuadraticProblemFormulation::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D, const Eigen::VectorXd& d,
    const Eigen::VectorXd& f, Eigen::VectorXd& x)
{
  // f = (Ax-b)' S (Ax-b) + x' W x = x'A'SAx - 2x'A'Sb + b'Sb + x'Wx.
  // This means minimizing f is equivalent to minimizing: 1/2 x'Qx + c'x,
  // where Q = A'SA + W and c = -A'Sb.
  // Adapted from 'simulationandcontrol' by Stelian Coros.

  int m = A.rows();
  int n = A.cols();
  x.setZero(n);
  OOQPEI_ASSERT_EQ(range_error, b.size(), m, "Vector b has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, S.rows(), m, "Matrix S has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, W.rows(), n, "Matrix W has wrong size.");

  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;

#if EIGEN_VERSION_AT_LEAST(3,2,92)
  Q_temp = A.transpose() * S * A + (Eigen::SparseMatrix<double, Eigen::RowMajor>)W.toDenseMatrix().sparseView();
#else
  Q_temp = A.transpose() * S * A + W.toDenseMatrix().sparseView();
#endif

  VectorXd c_temp = -A.transpose() * S * b;

  return OoqpEigenInterface::solve(Q_temp, c_temp, C, c, D, d, f, x);
}

bool QuadraticProblemFormulation::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D, const Eigen::VectorXd& d,
    const Eigen::VectorXd& f, Eigen::VectorXd& x)
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd c;
  return solve(A, S, b, W, C, c, D, d, f, x);
}

} /* namespace ooqpei */
