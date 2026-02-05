#pragma once

#include <memory>

#include <pr2_wholebody_interface/wholebody_precomputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace pr2_wholebody_interface{
using namespace ocs2;
class PR2WBCSelfCollisionConstraint final : public SelfCollisionConstraint {
public:
  PR2WBCSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~PR2WBCSelfCollisionConstraint() override = default;
  PR2WBCSelfCollisionConstraint(const PR2WBCSelfCollisionConstraint& other) = default;
  PR2WBCSelfCollisionConstraint* clone() const { return new PR2WBCSelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<PR2WBCPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace mobile_manipulator