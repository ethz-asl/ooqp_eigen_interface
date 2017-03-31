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
* @file    OoqpEigenInterface.cpp
* @author  Péter Fankhauser, Christian Gehring
* @date    Aug 13, 2013
*/

#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include <stdexcept>
#include "ooqp_eigen_interface/ooqpei_assert_macros.hpp"
#include "ooqp/QpGenData.h"
#include "ooqp/QpGenVars.h"
#include "ooqp/QpGenResiduals.h"
#include "ooqp/GondzioSolver.h"
#include "ooqp/QpGenSparseMa27.h"
#include "ooqp/Status.h"
#include "ooqp_eigen_interface/ooqpei_numerical_comparisons.hpp"

using namespace Eigen;
using namespace std;
using namespace ooqpei;

namespace ooqpei {

bool OoqpEigenInterface::isInDebugMode_ = false;

bool OoqpEigenInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                               const Eigen::VectorXd& c,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                               const Eigen::VectorXd& b,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                               const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                               const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                               Eigen::VectorXd& x,
                               const bool ignoreUnknownError)
{
  // Initialize.
  int nx = Q.rows();  // nx is the number of primal variables (x).
  OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
  x.setZero(nx);

  // Make copies of variables that are changed.
  auto ccopy(c);
  auto Acopy(A);
  auto bcopy(b);
  auto Ccopy(C);

  // Make sure Q is in lower triangular form (Q is symmetric).
  // Refer to OOQP user guide section 2.2 (p. 11).
  // TODO Check if Q is really symmetric.
  SparseMatrix<double, Eigen::RowMajor> Q_triangular = Q.triangularView<Lower>();

  if (isInDebugMode()) printProblemFormulation(Q_triangular, ccopy, Acopy, bcopy, Ccopy, d, f, l, u);

  // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual).
  Q_triangular.makeCompressed();
  Acopy.makeCompressed();
  Ccopy.makeCompressed();

  // Check matrices.
  OOQPEI_ASSERT_EQ(range_error, ccopy.size(), nx, "Vector c has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, l.size(), nx, "Vector l has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, u.size(), nx, "Vector u has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, bcopy.size(), Acopy.rows(), "Vector b has wrong size.");
  if (Acopy.size() > 0) OOQPEI_ASSERT_EQ(range_error, Acopy.cols(), nx, "Matrix A has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, d.size(), Ccopy.rows(), "Vector d has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, f.size(), Ccopy.rows(), "Vector f has wrong size.");
  if (Ccopy.size() > 0) OOQPEI_ASSERT_EQ(range_error, Ccopy.cols(), nx, "Matrix C has wrong size.");

  // Determine which limits are active and which are not.
  // Refer to OOQP user guide section 2.2 (p. 10).
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
  VectorXd lowerLimitForX;
  VectorXd upperLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForInequalityConstraints;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForInequalityConstraints;
  VectorXd lowerLimitForInequalityConstraints;
  VectorXd upperLimitForInequalityConstraints;

  generateLimits(l, u, useLowerLimitForX, useUpperLimitForX,
                 lowerLimitForX, upperLimitForX);
  generateLimits(d, f, useLowerLimitForInequalityConstraints,
                 useUpperLimitForInequalityConstraints,
                 lowerLimitForInequalityConstraints,
                 upperLimitForInequalityConstraints);

  if (isInDebugMode())
  {
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR X" << endl;
    printLimits(useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX);
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR INEQUALITY CONSTRAINTS" << endl;
    printLimits(useLowerLimitForInequalityConstraints,
                useUpperLimitForInequalityConstraints,
                lowerLimitForInequalityConstraints,
                upperLimitForInequalityConstraints);
  }

  // Setting up OOQP solver
  // Refer to OOQP user guide section 2.3 (p. 14).

  // Initialize new problem formulation.
  int my = bcopy.size();
  int mz = lowerLimitForInequalityConstraints.size();
  int nnzQ = Q_triangular.nonZeros();
  int nnzA = Acopy.nonZeros();
  int nnzC = Ccopy.nonZeros();

  QpGenSparseMa27 * qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);

  // Fill in problem data.
  double* cp   = &ccopy.coeffRef(0);
  int* krowQ   =  Q_triangular.outerIndexPtr();
  int* jcolQ   =  Q_triangular.innerIndexPtr();
  double* dQ   =  Q_triangular.valuePtr();
  double* xlow = &lowerLimitForX.coeffRef(0);
  char* ixlow  = &useLowerLimitForX.coeffRef(0);
  double* xupp = &upperLimitForX.coeffRef(0);
  char* ixupp  = &useUpperLimitForX.coeffRef(0);
  int* krowA   =  Acopy.outerIndexPtr();
  int* jcolA   =  Acopy.innerIndexPtr();
  double* dA   =  Acopy.valuePtr();
  double* bA   = &bcopy.coeffRef(0);
  int* krowC   =  Ccopy.outerIndexPtr();
  int* jcolC   =  Ccopy.innerIndexPtr();
  double* dC   =  Ccopy.valuePtr();
  double* clow = &lowerLimitForInequalityConstraints.coeffRef(0);
  char* iclow  = &useLowerLimitForInequalityConstraints.coeffRef(0);
  double* cupp = &upperLimitForInequalityConstraints.coeffRef(0);
  char* icupp  = &useUpperLimitForInequalityConstraints.coeffRef(0);

  QpGenData * prob = (QpGenData *) qp->makeData(cp, krowQ, jcolQ, dQ,
                                                xlow, ixlow, xupp, ixupp,
                                                krowA, jcolA, dA, bA,
                                                krowC, jcolC, dC,
                                                clow, iclow, cupp, icupp);

  // Create object to store problem variables.
  QpGenVars* vars = (QpGenVars*) qp->makeVariables(prob);
//  if (isInDebugMode()) prob->print(); // Matrices are printed as [index_x, index_y, value]

  // Create object to store problem residual data.
  QpGenResiduals* resid = (QpGenResiduals*) qp->makeResiduals(prob);

  // Create solver object.
  GondzioSolver* s = new GondzioSolver(qp, prob);
  if (isInDebugMode()) s->monitorSelf();

  // Solve.
  int status = s->solve(prob, vars, resid);

  if ((status == SUCCESSFUL_TERMINATION) || (ignoreUnknownError && (status == UNKNOWN)))
    vars->x->copyIntoArray(&x.coeffRef(0));

  if(isInDebugMode()) printSolution(status, x);
  //vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

  delete s;
  delete resid;
  delete vars;
  delete prob;
  delete qp;

  return ((status == SUCCESSFUL_TERMINATION) || (ignoreUnknownError && (status == UNKNOWN)));
}

bool OoqpEigenInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                               const Eigen::VectorXd& c,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                               const Eigen::VectorXd& b,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                               const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                               Eigen::VectorXd& x,
                               const bool ignoreUnknownError)
{
  int nx = Q.rows();
  VectorXd u = std::numeric_limits<double>::max() * VectorXd::Ones(nx);
  VectorXd l = (-u.array()).matrix();
  return solve(Q, c, A, b, C, d, f, l, u, x, ignoreUnknownError);
}

bool OoqpEigenInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                               const Eigen::VectorXd& c,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                               const Eigen::VectorXd& b,
                               const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                               Eigen::VectorXd& x,
                               const bool ignoreUnknownError)
{
  SparseMatrix<double, Eigen::RowMajor> C;
  VectorXd d, f;
  return solve(Q, c, A, b, C, d, f, l, u, x, ignoreUnknownError);
}

bool OoqpEigenInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                               const Eigen::VectorXd& c,
                               const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                               const Eigen::VectorXd& f,
                               Eigen::VectorXd& x,
                               const bool ignoreUnknownError)
{
  SparseMatrix<double, Eigen::RowMajor> A;
  VectorXd b;
  VectorXd d = -std::numeric_limits<double>::max() * VectorXd::Ones(C.rows());
  return solve(Q, c, A, b, C, d, f, x, ignoreUnknownError);
}

bool OoqpEigenInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                               const Eigen::VectorXd& c,
                               Eigen::VectorXd& x,
                               const bool ignoreUnknownError)
{
  SparseMatrix<double, Eigen::RowMajor> A, C;
  VectorXd b, d, f;
  return solve(Q, c, A, b, C, d, f, x, ignoreUnknownError);
}

void OoqpEigenInterface::generateLimits(
    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit)
{
  int n = l.size();
  useLowerLimit.setConstant(n, 1);
  useUpperLimit.setConstant(n, 1);
  lowerLimit = l;
  upperLimit = u;

  for (int i = 0; i < n; i++)
  {
    if (ooqpei::approximatelyEqual(l(i), -std::numeric_limits<double>::max()))
    {
      useLowerLimit(i) = 0;
      lowerLimit(i) = 0.0;
    }
    if (ooqpei::approximatelyEqual(u(i), std::numeric_limits<double>::max()))
    {
      useUpperLimit(i) = 0;
      upperLimit(i) = 0.0;
    }
  }
}

void OoqpEigenInterface::printProblemFormulation(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A, const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C, const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
  cout << "-------------------------------" << endl;
  cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u" << endl << endl;
  cout << "Q (triangular) << " << endl << MatrixXd(Q) << endl;
  cout << "c << " << c.transpose() << endl;
  cout << "A << " << endl << MatrixXd(A) << endl;
  cout << "b << " << b.transpose() << endl;
  cout << "C << " << endl << MatrixXd(C) << endl;
  cout << "d << " << d.transpose() << endl;
  cout << "f << " << f.transpose() << endl;
  cout << "l << " << l.transpose() << endl;
  cout << "u << " << u.transpose() << endl;
}

void OoqpEigenInterface::printLimits(
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit)
{
  cout << "useLowerLimit << " << std::boolalpha << useLowerLimit.cast<bool>().transpose() << endl;
  cout << "lowerLimit << " << lowerLimit.transpose() << endl;
  cout << "useUpperLimit << " << std::boolalpha << useUpperLimit.cast<bool>().transpose() << endl;
  cout << "upperLimit << " << upperLimit.transpose() << endl;
}

void OoqpEigenInterface::printSolution(const int status, const Eigen::VectorXd& x)
{
  if (status == 0)
  {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Ok, ended with status " << status << "."<< endl;
    cout << "x << " << x.transpose() << endl;
  }
  else
  {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Error, ended with status " << status << "." << endl;
  }
}

} /* namespace ooqpei */
