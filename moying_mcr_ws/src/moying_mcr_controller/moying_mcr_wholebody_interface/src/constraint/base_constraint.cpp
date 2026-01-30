#include "mmco_dual_interface/constraint/base_constraint.h"
#include "mmco_dual_interface/mmco_dual_precomputation.h"
#include <Eigen/Geometry>
#include <angles/angles.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace mmco_dual_interface{
using namespace ocs2;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using quaternion_t = Eigen::Quaternion<scalar_t>;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BaseConstraint::BaseConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const ReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager) {
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[BaseConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t BaseConstraint::getNumConstraints(scalar_t time) const {
  return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t BaseConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MMCODualPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

  vector_t constraint(6);
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
  constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation BaseConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MMCODualPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

  auto approximation = VectorFunctionLinearApproximation(6, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;

  const auto eeOrientationError =
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  approximation.f.tail<3>() = eeOrientationError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;

  return approximation;
}

std::pair<vector_t, quaternion_t>  BaseConstraint::interpolateEndEffectorPose(ocs2::scalar_t time) const{
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index];
    const auto& rhs = stateTrajectory[index + 1];
    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.segment(14,3) + (1.0 - alpha) * rhs.segment(14,3);
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // stateTrajectory.size() == 1
    ocs2::vector_t hs = stateTrajectory.front();
    ocs2::vector_t line_hs = hs.tail<7>();
    position = line_hs.head<3>();
    orientation = quaternion_t(line_hs.tail<4>());
  }



  return {position, orientation};
}

}  // namespace mobile_manipulator