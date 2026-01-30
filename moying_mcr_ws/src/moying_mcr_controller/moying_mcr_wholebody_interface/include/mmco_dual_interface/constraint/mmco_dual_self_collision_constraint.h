#pragma once

#include <memory>

#include <mmco_dual_interface/mmco_dual_precomputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace mmco_dual_interface{
using namespace ocs2;
class MMCODualSelfCollisionConstraint final : public SelfCollisionConstraint {
public:
  MMCODualSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~MMCODualSelfCollisionConstraint() override = default;
  MMCODualSelfCollisionConstraint(const MMCODualSelfCollisionConstraint& other) = default;
  MMCODualSelfCollisionConstraint* clone() const { return new MMCODualSelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<MMCODualPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace mobile_manipulator