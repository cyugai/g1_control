//
// Created by jialong xue on 23-1-29.
//

#pragma once

#include <ocs2_self_collision/SelfCollisionConstraint.h>

#include "g1_interface/g1PreComputation.h"

namespace ocs2 {
namespace g1 {
    using namespace ocs2;

    class LeggedSelfCollisionConstraint final : public SelfCollisionConstraint {
    public:
        LeggedSelfCollisionConstraint(const CentroidalModelPinocchioMapping& mapping, PinocchioGeometryInterface pinocchioGeometryInterface,
                                      scalar_t minimumDistance)
                : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
        ~LeggedSelfCollisionConstraint() override = default;
        LeggedSelfCollisionConstraint(const LeggedSelfCollisionConstraint& other) = default;
        LeggedSelfCollisionConstraint* clone() const override { return new LeggedSelfCollisionConstraint(*this); }

        const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
            return cast<g1PreComputation>(preComputation).getPinocchioInterface();
        }
    };

}  // namespace legged
}  // namespace ocs2
