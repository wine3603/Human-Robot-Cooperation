#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "pr2_wholebody_interface/wholebody_module_info.h"
#include "pr2_wholebody_interface/wholebody_precomputation.h"
#include "pr2_wholebody_interface/wholebody_pinocchio_mapping.h"

namespace pr2_wholebody_interface{

/** Callback for caching and reference update */
class PR2WBCPreComputation : public PreComputation {
 public:
  PR2WBCPreComputation(ocs2::PinocchioInterface pinocchioInterface, const PR2WBCModuleInfo& info);

  ~PR2WBCPreComputation() override = default;

  PR2WBCPreComputation(const PR2WBCPreComputation& rhs) = delete;
  PR2WBCPreComputation* clone() const override;

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
  void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

  ocs2::PinocchioInterface& getPinocchioInterface() {return pinocchioInterface_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  PR2WBCPinocchioMapping pinocchioMapping_;
};

}  // namespace mobile_manipulator