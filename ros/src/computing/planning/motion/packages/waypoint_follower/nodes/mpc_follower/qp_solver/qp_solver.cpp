/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mpc_follower/qp_solver/qp_solver.h"
#include <chrono>

QPSolverEigenLeastSquare::QPSolverEigenLeastSquare(){};
void QPSolverEigenLeastSquare::init(const int max_iter){};
bool QPSolverEigenLeastSquare::solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U)
{
     if (std::fabs(Hmat.determinant()) < 1.0E-9)
          return false;

     U = -Hmat.inverse() * fvec;

     return true;
}

QPSolverEigenLeastSquareLLT::QPSolverEigenLeastSquareLLT(){};
void QPSolverEigenLeastSquareLLT::init(const int max_iter){};
bool QPSolverEigenLeastSquareLLT::solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U)
{
     if (std::fabs(Hmat.determinant()) < 1.0E-9)
          return false;

     U = -Hmat.llt().solve(fvec);

     return true;
};

QPSolverQpoasesHotstart::QPSolverQpoasesHotstart(){};
void QPSolverQpoasesHotstart::init(const int max_iter)
{
     is_init_ = true;
     max_iter_ = max_iter;
}

bool QPSolverQpoasesHotstart::solve(const Eigen::MatrixXd &Hmat, const Eigen::MatrixXd &fvec, const double &max_u, Eigen::VectorXd &U)
{
     USING_NAMESPACE_QPOASES

     int max_iter = max_iter_;

     const int kNumOfMatrixElements = Hmat.rows() * Hmat.cols();
     double h_matrix[kNumOfMatrixElements];

     const int kNumOfoffsetRows = fvec.rows();
     double g_matrix[kNumOfoffsetRows];

     double lower_bound[kNumOfoffsetRows];
     double upper_bound[kNumOfoffsetRows];

     double result[kNumOfoffsetRows];
     U = Eigen::VectorXd::Zero(kNumOfoffsetRows);

     Eigen::MatrixXd Aconstraint = Eigen::MatrixXd::Identity(kNumOfoffsetRows, kNumOfoffsetRows);
     double a_constraint_matirx[kNumOfMatrixElements];

     int index = 0;

     for (int r = 0; r < Hmat.rows(); ++r)
     {
          g_matrix[r] = fvec(r, 0);
          for (int c = 0; c < Hmat.cols(); ++c)
          {
               h_matrix[index] = Hmat(r, c);
               a_constraint_matirx[index] = Aconstraint(r, c);
               index++;
          }
     }

     for (int i = 0; i < kNumOfoffsetRows; ++i)
     {
          lower_bound[i] = -max_u;
          upper_bound[i] = max_u;
     }

     solver_.setPrintLevel(qpOASES::PL_NONE);

     if (is_init_)
     {
          solver_ = qpOASES::SQProblem(kNumOfoffsetRows, kNumOfoffsetRows);
          auto ret = solver_.init(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, max_iter);
          if (ret != SUCCESSFUL_RETURN)
               return false;
     }
     else
     {
          auto ret = solver_.hotstart(h_matrix, g_matrix, a_constraint_matirx, lower_bound, upper_bound, lower_bound, upper_bound, max_iter);
          if (ret != SUCCESSFUL_RETURN)
               return false;
     }
     is_init_ = false;

     solver_.getPrimalSolution(result);

     for (int i = 0; i < kNumOfoffsetRows; ++i)
     {
          U(i) = result[i];
     }

     return true;
}