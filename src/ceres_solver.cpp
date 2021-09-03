// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)
//
// Cost function for a 2D pose graph formulation.

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "karto_graph_slam/ceres_solver/ceres_solver.h"
#include "karto_graph_slam/ceres_solver/angle_local_parameterization.h"
#include "karto_graph_slam/ceres_solver/pose_graph_2d_error_term.h"

// 指定角度相加时使用的方法
CeresSolver::CeresSolver() : loss_function_(nullptr), angle_local_parameterization_(AngleLocalParameterization::Create())
{
  first_optimize_ = true;
}

CeresSolver::~CeresSolver()
{
  if (loss_function_)
    delete loss_function_;
  if (angle_local_parameterization_)
    delete angle_local_parameterization_;
}

void CeresSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector &CeresSolver::GetCorrections() const
{
  return corrections_;
}

// 添加节点
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  int pose_id = pVertex->GetObject()->GetUniqueId();
  Pose2d pose2d;
  pose2d.x = pose.GetX();
  pose2d.y = pose.GetY();
  pose2d.yaw_radians = pose.GetHeading();
  poses_[pose_id] = pose2d;

  ROS_DEBUG("[ceres] AddNode %d", pVertex->GetObject()->GetUniqueId());
}

// 添加约束
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
{
  karto::LocalizedRangeScan *pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan *pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double, 3, 3> info;
  info(0, 0) = precisionMatrix(0, 0);
  info(0, 1) = info(1, 0) = precisionMatrix(0, 1);
  info(0, 2) = info(2, 0) = precisionMatrix(0, 2);
  info(1, 1) = precisionMatrix(1, 1);
  info(1, 2) = info(2, 1) = precisionMatrix(1, 2);
  info(2, 2) = precisionMatrix(2, 2);
  Eigen::Vector3d measurement(diff.GetX(), diff.GetY(), diff.GetHeading());

  Constraint2d constraint2d;
  constraint2d.id_begin = pSource->GetUniqueId();
  constraint2d.id_end = pTarget->GetUniqueId();
  constraint2d.x = measurement(0);
  constraint2d.y = measurement(1);
  constraint2d.yaw_radians = measurement(2);
  constraint2d.information = info;
  constraints_.push_back(constraint2d);

  ROS_DEBUG("[ceres] AddConstraint %d  %d", pSource->GetUniqueId(), pTarget->GetUniqueId());
}

// 优化求解
void CeresSolver::Compute()
{
  corrections_.clear();

  std::cout << "initial cost: " << summary_.initial_cost << std::endl;
  std::cout << "final cost: " << summary_.final_cost << std::endl;

  if(summary_.final_cost < 0.1 && !first_optimize_)
  {
    std::cout << "Cost is very small, not need to optimize" << std::endl;
    return;
  }

  ROS_INFO("[ceres] Calling ceres for loop closure");
  ceres::Problem problem;
  BuildOptimizationProblem(constraints_, &poses_, &problem);

  // ROS_INFO("[ceres] Finished build optimization problem");

  SolveOptimizationProblem(&problem);
  ROS_INFO("[ceres] Finished ceres for loop closure\n");

  for (std::map<int, Pose2d>::const_iterator pose_iter = poses_.begin(); pose_iter != poses_.end(); ++pose_iter)
  {
    karto::Pose2 pose(pose_iter->second.x, pose_iter->second.y, pose_iter->second.yaw_radians);
    corrections_.push_back(std::make_pair(pose_iter->first, pose));
  }
}

/**
 * @brief 从位姿图约束构造非线性最小二乘优化问题
 * 
 * @param constraints 位姿图约束
 * @param poses 节点位姿
 * @param problem 优化问题
 */
void CeresSolver::BuildOptimizationProblem(const std::vector<Constraint2d> &constraints, std::map<int, Pose2d> *poses,
                                           ceres::Problem *problem)
{
  std::cout << "Begin to build optimization problem" << std::endl;
  assert(poses != NULL);
  // std::cout << "11111111111111111111111111111111111" << std::endl;
  assert(problem != NULL);
  // std::cout << "22222222222222222222222222222222222" << std::endl;
  if (constraints.empty())
  {
    std::cout << "No constraints, no problem to optimize.";
    return;
  }

  // std::cout << "33333333333333333333333333333333333" << std::endl;

  for (std::vector<Constraint2d>::const_iterator constraints_iter = constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter)
  {
    const Constraint2d &constraint = *constraints_iter;

    std::map<int, Pose2d>::iterator pose_begin_iter = poses->find(constraint.id_begin);
    assert(pose_begin_iter != poses->end());
    // std::cout << "44444444444444444444444444444444444444" << std::endl;
    std::map<int, Pose2d>::iterator pose_end_iter = poses->find(constraint.id_end);
    assert(pose_end_iter != poses->end());
    // std::cout << "55555555555555555555555555555555555555" << std::endl;

    // 对information开根号
    const Eigen::Matrix3d sqrt_information = constraint.information.llt().matrixL();

    // std::cout << "66666666666666666666666666666666666666" << std::endl;

    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        PoseGraph2dErrorTerm::Create(constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);

    // std::cout << "7777777777777777777777777777777777777" << std::endl;

    problem->AddResidualBlock(cost_function, loss_function_,
                              &pose_begin_iter->second.x,
                              &pose_begin_iter->second.y,
                              &pose_begin_iter->second.yaw_radians,
                              &pose_end_iter->second.x,
                              &pose_end_iter->second.y,
                              &pose_end_iter->second.yaw_radians);

    // std::cout << "888888888888888888888888888888888888888" << std::endl;
    
    try
    {
      problem->SetParameterization(&pose_begin_iter->second.yaw_radians, angle_local_parameterization_);
      problem->SetParameterization(&pose_end_iter->second.yaw_radians, angle_local_parameterization_);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    
    // std::cout << "999999999999999999999999999999999999999" << std::endl;
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
  assert(pose_start_iter != poses->end());
  // std::cout << "1010101010101010101010101010101010101010" << std::endl;
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);

  std::cout << "Finish build optimization problem" << std::endl;
}

// Returns true if the solve was successful.
bool CeresSolver::SolveOptimizationProblem(ceres::Problem *problem)
{
  assert(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.function_tolerance = 1e-16;
  options.gradient_tolerance = 1e-16;
  // options.minimizer_progress_to_stdout = true;


  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  first_optimize_ = false;

  summary_ = summary;

  std::cout << summary.BriefReport() << '\n';
  return summary.IsSolutionUsable();
}

void CeresSolver::getGraph(std::vector<Constraint2d>& constraints, std::map<int, Pose2d>& poses)
{
  
  for (std::map<int, Pose2d>::const_iterator pose_iter = poses_.begin(); pose_iter != poses_.end(); ++pose_iter)
  {
    Pose2d pose_2d;
    pose_2d.x = pose_iter->second.x;
    pose_2d.y = pose_iter->second.y;
    pose_2d.yaw_radians = pose_iter->second.yaw_radians;

    poses.insert(std::make_pair(pose_iter->first, pose_2d));
  }

  for (std::vector<Constraint2d>::const_iterator constraints_iter = constraints_.begin(); constraints_iter != constraints_.end(); ++constraints_iter)
  {
    const Constraint2d &constraint = *constraints_iter;

    Constraint2d constraint2d;
    constraint2d.id_begin = constraint.id_begin;
    constraint2d.id_end = constraint.id_end;
    constraint2d.x = constraint.x;
    constraint2d.y = constraint.y;
    constraint2d.yaw_radians = constraint.yaw_radians;
    constraint2d.information = constraint.information;
    constraints.push_back(constraint2d);

  }

}
