#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "mmco_dual_interface/mmco_dual_precomputation.h"
#include "mmco_dual_interface/mmco_dual_module_info.h"

namespace mmco_dual_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MMCODualPreComputation::MMCODualPreComputation(PinocchioInterface pinocchioInterface, const MMCODualModuleInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MMCODualPreComputation* MMCODualPreComputation::clone() const {
  return new MMCODualPreComputation(pinocchioInterface_, pinocchioMapping_.getMMCODualModuleInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MMCODualPreComputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) {
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
void MMCODualPreComputation::requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) {
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

}  // namespace mmco_dual_interface