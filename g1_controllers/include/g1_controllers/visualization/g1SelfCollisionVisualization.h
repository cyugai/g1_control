//
// Created by qiayuan on 23-1-30.
//

#pragma once
#include "rclcpp/rclcpp.hpp"

#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <utility>

namespace ocs2 {
namespace g1 {


class g1SelfCollisionVisualization : public GeometryInterfaceVisualization {
 public:
  g1SelfCollisionVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping, rclcpp::Node::SharedPtr &node, scalar_t maxUpdateFrequency = 50.0)
      : mappingPtr_(mapping.clone()),
        GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), "odom"),
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}
  void update(const SystemObservation& observation) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
      lastTime_ = observation.time;

      publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
    }
  }

 private:
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace g1
}  // namespace ocs2
