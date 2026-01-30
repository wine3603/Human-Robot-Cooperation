#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h>

namespace pr2_wholebody_interface{
using namespace ocs2;
class BaseConstraint final : public StateConstraint {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  BaseConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager);
  ~BaseConstraint() override = default;
  BaseConstraint* clone() const override { return new BaseConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_); }

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

 private:
  BaseConstraint(const BaseConstraint& other) = default;
  std::pair<vector_t, quaternion_t> interpolateEndEffectorPose(scalar_t time) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const ReferenceManager* referenceManagerPtr_;
};

}  // namespace pr2_wholebody_interface