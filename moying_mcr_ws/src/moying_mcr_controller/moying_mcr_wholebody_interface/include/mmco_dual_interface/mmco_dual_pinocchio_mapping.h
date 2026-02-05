#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "mmco_dual_interface/mmco_dual_module_info.h"
namespace mmco_dual_interface{
using namespace ocs2;
template<typename SCALAR>
class MMCODualPinocchioMappingTpl;

using MMCODualPinocchioMapping = MMCODualPinocchioMappingTpl<ocs2::scalar_t>;
using MMCODualPinocchioMappingCppAd = MMCODualPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class MMCODualPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit MMCODualPinocchioMappingTpl(MMCODualModuleInfo info);

  ~MMCODualPinocchioMappingTpl() override = default;
  MMCODualPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const MMCODualModuleInfo& getMMCODualModuleInfo() const { return moduleInfo_; }
private:
  MMCODualPinocchioMappingTpl(const MMCODualPinocchioMappingTpl& rhs) = default;

  const MMCODualModuleInfo moduleInfo_;
};

}