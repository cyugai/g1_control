//
// Created by qiayuan on 2022/6/28.
//
//
// Ref: https://github.com/bernhardpg/quadruped_locomotion
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "g1_wbc/Task.h"

#include <memory>

namespace ocs2
{
namespace g1
{
// Hierarchical Optimization Quadratic Program
class HoQp
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  HoQp(Task task, HoQpPtr higherProblem);

  matrix_t getStackedZMatrix() const
  {
    return stackedZ_;
  }

  Task getStackedTasks() const
  {
    return stackedTasks_;
  }

  vector_t getStackedSlackSolutions() const
  {
    return stackedSlackVars_;
  }

  vector_t getSolutions() const
  {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  size_t getSlackedNumVars() const
  {
    return stackedTasks_.d_.rows();
  }

private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  void buildZMatrix();
  void stackSlackSolutions();

  Task task_, stackedTasksPrev_, stackedTasks_;
  HoQpPtr higherProblem_;

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};
  matrix_t stackedZPrev_, stackedZ_;
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  matrix_t eyeNvNv_;
  matrix_t zeroNvNx_;
};

}  // namespace g1
}  // namespace ocs2
