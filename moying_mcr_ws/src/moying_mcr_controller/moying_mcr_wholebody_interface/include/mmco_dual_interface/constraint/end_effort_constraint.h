#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace mmco_dual_interface{
using namespace ocs2;
class EndEffectorConstraint final : public StateConstraint {
public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  EndEffectorConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics, const ocs2::ReferenceManager& referenceManager,std::string endEffectorName);
  ~EndEffectorConstraint() override = default;
  EndEffectorConstraint* clone() const override { return new EndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_,endEffectorName_); }

  size_t getNumConstraints(scalar_t time) const override;
  ocs2::vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

private:
  EndEffectorConstraint(const EndEffectorConstraint& other) = default;
  virtual std::pair<vector_t, quaternion_t> interpolateEndEffectorPose(scalar_t time) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  ocs2::PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::string endEffectorName_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const ocs2::ReferenceManager* referenceManagerPtr_;
};

}  // namespace mmco_dual_interface