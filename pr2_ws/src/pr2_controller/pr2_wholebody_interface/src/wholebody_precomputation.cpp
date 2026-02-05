#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "pr2_wholebody_interface/wholebody_precomputation.h"
#include "pr2_wholebody_interface/wholebody_module_info.h"

namespace pr2_wholebody_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PR2WBCPreComputation::PR2WBCPreComputation(PinocchioInterface pinocchioInterface, const PR2WBCModuleInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PR2WBCPreComputation* PR2WBCPreComputation::clone() const {
  return new PR2WBCPreComputation(pinocchioInterface_, pinocchioMapping_.getPR2WBCModuleInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PR2WBCPreComputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) {
  if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(ocs2::Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateGlobalPlacements(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PR2WBCPreComputation::requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) {
  if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(ocs2::Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace pr2_wholebody_interface