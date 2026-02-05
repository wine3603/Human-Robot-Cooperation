#include "mmco_dual_interface/mmco_dual_pinocchio_mapping.h"
#include "mmco_dual_interface/mmco_dual_module_info.h"

namespace mmco_dual_interface{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MMCODualPinocchioMappingTpl<SCALAR>::MMCODualPinocchioMappingTpl(MMCODualModuleInfo info)
    : moduleInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MMCODualPinocchioMappingTpl<SCALAR>* MMCODualPinocchioMappingTpl<SCALAR>::clone() const {
  return new MMCODualPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MMCODualPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MMCODualPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {
  const int armStateDim  = moduleInfo_.leftArmDim  + moduleInfo_.rightArmDim ;
  vector_t vPinocchio = vector_t::Zero(armStateDim);
  vPinocchio <<  input.tail(armStateDim);
  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MMCODualPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  // set jacobian model based on model type
  const int armDim  = moduleInfo_.leftArmDim  + moduleInfo_.rightArmDim ;
  matrix_t dfdu(Jv.rows(),armDim);
  // clang-format on
  dfdu.template rightCols(armDim) = Jv.template rightCols(armDim);
  return {Jq, dfdu};
}

// explicit template instantiation
template class mmco_dual_interface::MMCODualPinocchioMappingTpl<ocs2::scalar_t>;
template class mmco_dual_interface::MMCODualPinocchioMappingTpl<ocs2::ad_scalar_t>;
} // namespace