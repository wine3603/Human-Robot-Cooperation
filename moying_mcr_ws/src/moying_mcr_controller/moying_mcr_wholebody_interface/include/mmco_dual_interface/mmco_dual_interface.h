#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "mmco_dual_interface/factory_functions.h"
#include "mmco_dual_interface/mmco_dual_module_info.h"
#include "mmco_dual_interface/mmco_dual_precomputation.h"
#include "mmco_dual_interface/mmco_dual_pinocchio_mapping.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace mmco_dual_interface{
using namespace ocs2;
class MMCODualInterface final: public RobotInterface{
public:
  MMCODualInterface(const std::string& taskFile, const std::string& libraryFolder,
                                      const std::string& urdfFile);
  ~MMCODualInterface() override = default;
  const ocs2::vector_t & getInitializerState(){ return initialState_; }

  ocs2::ddp::Settings& ddpSettings(){ return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings(){ return mpcSettings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override {return problem_;}

  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override {return referenceManagerPtr_;}

  const ocs2::Initializer& getInitializer() const override {return *initializerPtr_;}

  const ocs2::RolloutBase& getRollout() const {return *rolloutPtr_;}

  const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_;}

  const MMCODualModuleInfo& getMMCODualModuelInfo() const {return mmcoDualModuleInfo_;}
private:
  std::unique_ptr<ocs2::StateInputCost> getQuadraticInputCost(const std::string& taskFile);
  std::unique_ptr<ocs2::StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries,const std::string endEffectorName);
  std::unique_ptr<ocs2::StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& urdfFile, const std::string& prefix, bool useCaching,
                                                        const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

  std::unique_ptr<ocs2::StateCost> getBaseConstraint(const PinocchioInterface& pinocchioInterface,const std::string& taskFile,
                                                      const std::string& prefix,bool usePreComputation, const std::string& libraryFolder,
                                                      bool recompileLibraries);

  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;

  ocs2::OptimalControlProblem problem_;
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
  std::unique_ptr<ocs2::PinocchioInterface> pinocchioNoBaseInterfacePtr_;
  MMCODualModuleInfo mmcoDualModuleInfo_;


  ocs2::vector_t initialState_;

};

}// namespacce