//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "g1_wbc/HierarchicalWbc.h"

#include "g1_wbc/HoQp.h"

namespace ocs2
{
namespace g1
{
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& rbdStateMeasured, size_t mode, scalar_t period)
{
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
               formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period);
  Task task2 = formulateContactForceTask(inputDesired) * 0.1 + formulateSwingLegTask() * 1;
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}

}  // namespace g1
}  // namespace ocs2

