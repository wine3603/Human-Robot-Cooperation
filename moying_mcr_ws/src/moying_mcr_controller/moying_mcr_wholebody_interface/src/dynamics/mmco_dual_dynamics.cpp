#include "mmco_dual_interface/dynamics/mmco_dual_dynamics.h"

namespace mmco_dual_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MMCODualDynamics::MMCODualDynamics(MMCODualModuleInfo info, const std::string& modelName,
                                                                         const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                                         bool recompileLibraries /*= true*/, bool verbose /*= true*/)
    : info_(std::move(info)) {
  this->initialize(info_.stateDim, info_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t MMCODualDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                               const ad_vector_t&) const {
  ad_vector_t dxdt(info_.stateDim);
  const auto theta = state(2);
  const auto vx = input(0);  // forward velocity in base frame
  const auto vy = input(1);  // forward velocity in base frame
  dxdt << cos(theta) * vx - sin(theta) * vy, sin(theta) * vx + cos(theta) * vy, input(2), input.tail(info_.rightArmDim + info_.leftArmDim);
  return dxdt;
}

}  // namespace mobile_manipulator