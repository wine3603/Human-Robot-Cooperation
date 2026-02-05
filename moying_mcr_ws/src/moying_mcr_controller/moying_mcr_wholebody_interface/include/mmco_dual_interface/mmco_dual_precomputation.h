#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "mmco_dual_interface/mmco_dual_module_info.h"
#include "mmco_dual_interface/mmco_dual_precomputation.h"
#include "mmco_dual_interface/mmco_dual_pinocchio_mapping.h"

namespace mmco_dual_interface{

/** Callback for caching and reference update */
class MMCODualPreComputation : public PreComputation {
 public:
  MMCODualPreComputation(ocs2::PinocchioInterface pinocchioInterface, const MMCODualModuleInfo& info);

  ~MMCODualPreComputation() override = default;

  MMCODualPreComputation(const MMCODualPreComputation& rhs) = delete;
  MMCODualPreComputation* clone() const override;

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
  void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

  ocs2::PinocchioInterface& getPinocchioInterface() {return pinocchioInterface_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  MMCODualPinocchioMapping pinocchioMapping_;
};

}  // namespace mobile_manipulator