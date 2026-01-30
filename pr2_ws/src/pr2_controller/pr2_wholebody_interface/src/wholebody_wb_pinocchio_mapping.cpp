#include "pr2_wholebody_interface/wholebody_wb_pinocchio_mapping.h"
#include "pr2_wholebody_interface/wholebody_module_info.h"

namespace pr2_wholebody_interface{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PR2WBCWBPinocchioMappingTpl<SCALAR>::PR2WBCWBPinocchioMappingTpl(PR2WBCModuleInfo info)
    : moduleInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PR2WBCWBPinocchioMappingTpl<SCALAR>* PR2WBCWBPinocchioMappingTpl<SCALAR>::clone() const {
  return new PR2WBCWBPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCWBPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCWBPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {
  vector_t vPinocchio = vector_t::Zero(moduleInfo_.stateDim);
  const auto theta = state(2);
  const auto vx = input(0);  // forward velocity in base frame
  const auto vy = input(1);  // forward velocity in base frame
  const int armStateDim = moduleInfo_.leftArmDim + moduleInfo_.rightArmDim + moduleInfo_.torsoLiftDim;
  vPinocchio << cos(theta) * vx + sin(theta) * vy, sin(theta) * vx - cos(theta)*vy, input(2), input.tail(armStateDim);

  std::cerr<<"vPinocchio的大小："<<vPinocchio.size()<<"维度======\n";
  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PR2WBCWBPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  // set jacobian model based on model type
  std::cerr<<"dfdu的大小:"<<Jv.rows()<<"输入维度"<<moduleInfo_.inputDim<<"======\n";
  matrix_t dfdu(Jv.rows(), moduleInfo_.inputDim);
  Eigen::Matrix<SCALAR, 3, 3> dvdu_base;
  const SCALAR theta = state(2);
  // clang-format off
  dvdu_base << cos(theta), sin(theta), SCALAR(0),
                sin(theta),-cos(theta),SCALAR(0),
                SCALAR(0),SCALAR(0), SCALAR(1.0);
  // clang-format on
  dfdu.template leftCols<3>() = Jv.template leftCols<3>() * dvdu_base;
  const int armDim  = moduleInfo_.leftArmDim  + moduleInfo_.rightArmDim + moduleInfo_.torsoLiftDim;
  dfdu.template rightCols(armDim) = Jv.template rightCols(armDim);
  // std::cout << "最终的Jq:\n" << Jq << "\n";
  // std::cout << "最终的dfdu:\n" << dfdu << "\n======\n";
  return {Jq, dfdu};
}

// explicit template instantiation
template class pr2_wholebody_interface::PR2WBCWBPinocchioMappingTpl<ocs2::scalar_t>;
template class pr2_wholebody_interface::PR2WBCWBPinocchioMappingTpl<ocs2::ad_scalar_t>;
} // namespace