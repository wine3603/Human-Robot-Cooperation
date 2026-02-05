#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "mmco_dual_interface/mmco_dual_module_info.h"
namespace mmco_dual_interface{
using namespace ocs2;
template<typename SCALAR>
class MMCODualWBPinocchioMappingTpl;

using MMCODualWBPinocchioMapping = MMCODualWBPinocchioMappingTpl<ocs2::scalar_t>;
using MMCODualWBPinocchioMappingCppAd = MMCODualWBPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class MMCODualWBPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit MMCODualWBPinocchioMappingTpl(MMCODualModuleInfo info);

  ~MMCODualWBPinocchioMappingTpl() override = default;
  MMCODualWBPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const MMCODualModuleInfo& getMMCODualModuleInfo() const { return moduleInfo_; }
private:
  MMCODualWBPinocchioMappingTpl(const MMCODualWBPinocchioMappingTpl& rhs) = default;

  const MMCODualModuleInfo moduleInfo_;
};

}