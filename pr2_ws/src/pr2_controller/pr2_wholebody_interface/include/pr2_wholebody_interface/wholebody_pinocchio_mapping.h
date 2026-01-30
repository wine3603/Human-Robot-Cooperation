#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "pr2_wholebody_interface/wholebody_module_info.h"
namespace pr2_wholebody_interface{
using namespace ocs2;
template<typename SCALAR>
class PR2WBCPinocchioMappingTpl;

using PR2WBCPinocchioMapping = PR2WBCPinocchioMappingTpl<ocs2::scalar_t>;
using PR2WBCPinocchioMappingCppAd = PR2WBCPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class PR2WBCPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit PR2WBCPinocchioMappingTpl(PR2WBCModuleInfo info);

  ~PR2WBCPinocchioMappingTpl() override = default;
  PR2WBCPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const PR2WBCModuleInfo& getPR2WBCModuleInfo() const { return moduleInfo_; }
private:
  PR2WBCPinocchioMappingTpl(const PR2WBCPinocchioMappingTpl& rhs) = default;

  const PR2WBCModuleInfo moduleInfo_;
};

}