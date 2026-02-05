#include "pr2_wholebody_interface/wholebody_pinocchio_mapping.h"
#include "pr2_wholebody_interface/wholebody_module_info.h"

namespace pr2_wholebody_interface{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PR2WBCPinocchioMappingTpl<SCALAR>::PR2WBCPinocchioMappingTpl(PR2WBCModuleInfo info)
    : moduleInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PR2WBCPinocchioMappingTpl<SCALAR>* PR2WBCPinocchioMappingTpl<SCALAR>::clone() const {
  return new PR2WBCPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {
  const int armStateDim  = moduleInfo_.leftArmDim  + moduleInfo_.rightArmDim + moduleInfo_.torsoLiftDim;
  vector_t vPinocchio = vector_t::Zero(armStateDim);
  vPinocchio <<  input.tail(armStateDim);
  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  // set jacobian model based on model type
  const int armDim  = moduleInfo_.leftArmDim  + moduleInfo_.rightArmDim + moduleInfo_.torsoLiftDim;
  matrix_t dfdu(Jv.rows(),armDim);
  // clang-format on
  dfdu.template rightCols(armDim) = Jv.template rightCols(armDim);
  return {Jq, dfdu};
}

// explicit template instantiation
template class pr2_wholebody_interface::PR2WBCPinocchioMappingTpl<ocs2::scalar_t>;
template class pr2_wholebody_interface::PR2WBCPinocchioMappingTpl<ocs2::ad_scalar_t>;
} // namespace