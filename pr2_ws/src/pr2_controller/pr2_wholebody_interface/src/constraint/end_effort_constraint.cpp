#include "pr2_wholebody_interface/constraint/end_effort_constraint.h"
#include "pr2_wholebody_interface/wholebody_precomputation.h"

#include <ocs2_core/misc/LinearInterpolation.h>
  

// #define mode 1

namespace pr2_wholebody_interface{
using namespace ocs2;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                                             const ocs2::ReferenceManager& referenceManager,std::string endEffectorName)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager),endEffectorName_(endEffectorName) {
  std::cerr << "endEffectorName_ " << endEffectorName_ << std::endl;
  if(endEffectorName_.find("ight") !=std::string::npos)
  {
    std::cerr << "right select====================" << std::endl;
  }
  else
  {
    std::cerr << "left select====================" << std::endl;
  }
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<ocs2::PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(ocs2::scalar_t time) const {
  return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<PR2WBCPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);

  vector_t constraint(6);
  

  #ifdef mode

  ocs2::vector_t arm_state(12);
  for(int i =0;i<12;i++)
    arm_state(i) = state(3+i);

  
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(arm_state).front() - desiredPositionOrientation.first;

  constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(arm_state, {desiredPositionOrientation.second}).front();
  #else
  constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;

  constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
  #endif
  // std::cerr<<endEffectorName_<<"_constraint.tail<3>() "<<endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front()<<std::endl;
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<PR2WBCPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  const auto desiredPositionOrientation = interpolateEndEffectorPose(time);


  auto approximation = ocs2::VectorFunctionLinearApproximation(6, state.rows(), 0);
  

  #ifdef mode
  matrix_t temp = matrix_t::Zero(3,15);
  ocs2::vector_t arm_state(12);
  for(int i =0;i<12;i++)
    arm_state(i) = state(3+i);
  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(arm_state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  temp.block<3,12>(0,3) = eePosition.dfdx;
  approximation.dfdx.topRows<3>() = temp;
  #else
  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;
  #endif


  #ifdef mode
  const auto eeOrientationError =
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(arm_state, {desiredPositionOrientation.second}).front();
  approximation.f.tail<3>() = eeOrientationError.f;
  temp.block<3,12>(0,3) = eeOrientationError.dfdx;
  approximation.dfdx.bottomRows<3>() = temp;
  #else
  const auto eeOrientationError =
      endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
  approximation.f.tail<3>() = eeOrientationError.f;
  approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;
  #endif


  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::interpolateEndEffectorPose(ocs2::scalar_t time) const -> std::pair<ocs2::vector_t, quaternion_t> {
  const auto targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto timeTrajectory = targetTrajectories.timeTrajectory;
  const auto stateTrajectory = targetTrajectories.stateTrajectory;

  ocs2::vector_t position;
  quaternion_t orientation;

  if (stateTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    ocs2::scalar_t alpha;
    std::tie(index, alpha) = ocs2::LinearInterpolation::timeSegment(time, timeTrajectory);

    auto lhs = stateTrajectory[index];
    auto rhs = stateTrajectory[index + 1];
    //right
    if(endEffectorName_.find("ight") !=std::string::npos)
    {
      lhs = lhs.segment(7,7);
      rhs = lhs.segment(7,7);
    }
    else
    {
      lhs = lhs.segment(0,7);
      rhs = lhs.segment(0,7);
    }
    const quaternion_t q_lhs(lhs.tail<4>());
    const quaternion_t q_rhs(rhs.tail<4>());

    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // stateTrajectory.size() == 1
    ocs2::vector_t hs = stateTrajectory.front();
    ocs2::vector_t line_hs = hs.tail<7>();
    if(endEffectorName_.find("ight") !=std::string::npos)
    {
      line_hs = hs.segment(7,7);
    }
    else
    {
      line_hs = hs.segment(0,7);
    }
    position = line_hs.head<3>();
    orientation = quaternion_t(line_hs.tail<4>());
  }

  return {position, orientation};
}

}  // namespace mobile_manipulator