#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "pr2_wholebody_interface/wholebody_module_info.h"
namespace pr2_wholebody_interface{
using namespace ocs2;
template<typename SCALAR>
class PR2WBCWBPinocchioMappingTpl;

using PR2WBCWBPinocchioMapping = PR2WBCWBPinocchioMappingTpl<ocs2::scalar_t>;
using PR2WBCWBPinocchioMappingCppAd = PR2WBCWBPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class PR2WBCWBPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit PR2WBCWBPinocchioMappingTpl(PR2WBCModuleInfo info);

  ~PR2WBCWBPinocchioMappingTpl() override = default;
  PR2WBCWBPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const PR2WBCModuleInfo& getPR2WBCModuleInfo() const { return moduleInfo_; }
private:
  PR2WBCWBPinocchioMappingTpl(const PR2WBCWBPinocchioMappingTpl& rhs) = default;

  const PR2WBCModuleInfo moduleInfo_;
};

}