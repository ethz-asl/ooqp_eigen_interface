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
* @file    QuadraticProblemFormulation.hpp
* @author  Péter Fankhauser
* @date    Aug 16, 2013
*/

#pragma once

#include<Eigen/Core>
#include<Eigen/SparseCore>

namespace ooqpei {

class QuadraticProblemFormulation
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and d <= Dx <= f
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    Eigen::VectorXd& x);

  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that d <= Dx <= f
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S, const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    Eigen::VectorXd& x);
};

} /* namespace ooqpei */
