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
* @file    OoqpEigenInterface.hpp
* @author  Péter Fankhauser, Christian Gehring
* @date    Aug 13, 2013
* @brief   Uses the Object Oriented QP solver package (OOQP) to solve
*          convex quadratic optimization problems of the type:
*          Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u
*          where Q is symmetric positive semidefinite (nxn), x is a vector (nx1),
*          A and C are (possibly null) matrices and b and d are vectors of appropriate dimensions.
*          We are using sparse matrices in the Harwell-Boeing row-major format.
*          Adapted from 'simulationandcontrol' by Stelian Coros.
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace ooqpei {

class OoqpEigenInterface
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f, and l <= x <= u.
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [in] l a vector (nx1)
   * @param [in] u a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x,
                    const bool ignoreUnknownError = false);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    Eigen::VectorXd& x,
                    const bool ignoreUnknownError = false);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and l <= x <= u.
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] l a vector (nx1)
   * @param [in] u a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x,
                    const bool ignoreUnknownError = false);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& f,
                    Eigen::VectorXd& x,
                    const bool ignoreUnknownError = false);

  /*!
   * Solve min 1/2 x' Q x + c' x
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    Eigen::VectorXd& x,
                    const bool ignoreUnknownError = false);

  /*!
   * Change to true to print debug information.
   * @return true if in debug mode
   */
  static bool isInDebugMode() { return isInDebugMode_; };
  static void setIsInDebugMode(bool isInDebugMode) {
    isInDebugMode_ = isInDebugMode;
  }

 private:
  /*!
   * Determine which limits are active and which are not.
   * @param [in]  l
   * @param [in]  u
   * @param [out] useLowerLimit
   * @param [out] useUpperLimit
   * @param [out] lowerLimit
   * @param [out] upperLimit
   */
  static void generateLimits(const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                      Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
                      Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
                      Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit);

  static void printProblemFormulation(
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f,
      const Eigen::VectorXd& l, const Eigen::VectorXd& u);

  static void printLimits(const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
                          const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
                          const Eigen::VectorXd& lowerLimit,
                          const Eigen::VectorXd& upperLimit);

  static void printSolution(const int status, const Eigen::VectorXd& x);

 private:
  static bool isInDebugMode_;
};

} /* namespace ooqpei */
