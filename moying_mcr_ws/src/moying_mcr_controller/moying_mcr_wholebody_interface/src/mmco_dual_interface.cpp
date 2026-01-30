
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

#include "mmco_dual_interface/mmco_dual_interface.h"

#include "mmco_dual_interface/constraint/end_effort_constraint.h"
#include "mmco_dual_interface/constraint/base_constraint.h"
#include "mmco_dual_interface/constraint/mmco_dual_self_collision_constraint.h"
#include "mmco_dual_interface/dynamics/mmco_dual_dynamics.h"
#include "mmco_dual_interface/factory_functions.h"
#include "mmco_dual_interface/mmco_dual_precomputation.h"
#include "mmco_dual_interface/mmco_dual_pinocchio_mapping.h"
#include "mmco_dual_interface/mmco_dual_wb_pinocchio_mapping.h"
#include "mmco_dual_interface/cost/quadratic_input_cost.h"
// Boost
#include <boost/filesystem/operations.hpp>



namespace mmco_dual_interface {
using namespace ocs2;
MMCODualInterface::MMCODualInterface(const std::string& taskFile, const std::string& libraryFolder,
                                      const std::string& urdfFile)
{
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[MMCODualInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MMCODualInterface] Task file not found: " + taskFilePath.string());
  }
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[MMCODualInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MMCODualInterface] URDF file not found: " + urdfFilePath.string());
  }
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[MMCODualInterface] Generated library path: " << libraryFolderPath << std::endl;



  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::vector<std::string> removeJointNames;
  ocs2::loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
  std::string baseFrame, eleFrame,ereFrame;
  ocs2::loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  ocs2::loadData::loadPtreeValue<std::string>(pt, eleFrame, "model_information.eleFrame", false);
  ocs2::loadData::loadPtreeValue<std::string>(pt, ereFrame, "model_information.ereFrame", false);
  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### Model: MMCO Dual ";
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
  std::cerr << "\n #### model_information.eleFrame: \"" << eleFrame << "\"" ;
  std::cerr << "\n #### model_information.ereFrame: \"" << ereFrame << "\"" << std::endl;
  std::cerr << " #### =============================================================================" << std::endl;
  
  pinocchioNoBaseInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterfaceWithoutBase(urdfFile,removeJointNames)));
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile,removeJointNames)));
  std::cerr << " #### =============================================================================" << std::endl;
  std::cerr << *pinocchioInterfacePtr_;
  mmcoDualModuleInfo_ = mmco_dual_interface::createMMCODualModuleInfo(*pinocchioInterfacePtr_,baseFrame, eleFrame,ereFrame);
  bool usePreComputation = true;
  bool recompileLibraries = true;
  std::cerr << "\n #### Model Settings:";
  ocs2::loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
  ocs2::loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  initialState_.setZero(mmcoDualModuleInfo_.stateDim);
  const int baseStateDim = mmcoDualModuleInfo_.stateDim - mmcoDualModuleInfo_.leftArmDim - mmcoDualModuleInfo_.rightArmDim;
  const int leftArmStateDim = mmcoDualModuleInfo_.leftArmDim;
  const int rightArmStateDim = mmcoDualModuleInfo_.rightArmDim;
  if (baseStateDim > 0) {
    ocs2::vector_t initialBaseState = ocs2::vector_t::Zero(baseStateDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "initialState.base", initialBaseState);
    initialState_.head(baseStateDim) = initialBaseState;
  }
  ocs2::vector_t initialLeftArmState = ocs2::vector_t::Zero(leftArmStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.leftArm", initialLeftArmState);
  initialState_.tail(leftArmStateDim) = initialLeftArmState;

  ocs2::vector_t initialRightArmState = ocs2::vector_t::Zero(rightArmStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.rightArm", initialRightArmState);
  initialState_.tail(rightArmStateDim) = initialRightArmState;

  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  ddpSettings_ = ocs2::ddp::loadSettings(taskFile,"ddp");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile,"mpc");

  referenceManagerPtr_.reset(new ocs2::ReferenceManager);
  // Cost
  problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

  // Constraints
  // joint limits constraint
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
  // end-effector state constraint
  //Left Arm
  // #define mode 1
  #ifdef mode
  problem_.stateSoftConstraintPtr->add("endLeftEffector", getEndEffectorConstraint(*pinocchioNoBaseInterfacePtr_, taskFile, "endLeftEffector",
                                                                                usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.eleFrame));
  problem_.finalSoftConstraintPtr->add("finalLeftEndEffector", getEndEffectorConstraint(*pinocchioNoBaseInterfacePtr_, taskFile, "finalEndLeftEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.eleFrame));
  //Right Arm
  problem_.stateSoftConstraintPtr->add("endRightEffector", getEndEffectorConstraint(*pinocchioNoBaseInterfacePtr_, taskFile, "endRightEffector",
                                                                                usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.ereFrame));
  problem_.finalSoftConstraintPtr->add("finalRightEndEffector", getEndEffectorConstraint(*pinocchioNoBaseInterfacePtr_, taskFile, "finalEndRightEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.ereFrame));
  #else
  problem_.stateSoftConstraintPtr->add("endLeftEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endLeftEffector",
                                                                                usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.eleFrame));
  problem_.finalSoftConstraintPtr->add("finalLeftEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndLeftEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.eleFrame));
  //Right Arm
  problem_.stateSoftConstraintPtr->add("endRightEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endRightEffector",
                                                                                usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.ereFrame));
  problem_.finalSoftConstraintPtr->add("finalRightEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndRightEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries,mmcoDualModuleInfo_.ereFrame));

  #endif
  problem_.stateSoftConstraintPtr->add("baseBase", getBaseConstraint(*pinocchioInterfacePtr_, taskFile, "endBase",
                                                                                usePreComputation, libraryFolder, recompileLibraries));
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  if (activateSelfCollision) {
    problem_.stateSoftConstraintPtr->add(
        "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
  }
  problem_.dynamicsPtr.reset(
          new MMCODualDynamics(mmcoDualModuleInfo_, "dynamics", libraryFolder, recompileLibraries, true));

  if (usePreComputation) {
    problem_.preComputationPtr.reset(new MMCODualPreComputation(*pinocchioInterfacePtr_, mmcoDualModuleInfo_));
  }

  // Rollout
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));
  std::cerr << " #### =============================================================================" << std::endl;

  // Initialization
  initializerPtr_.reset(new DefaultInitializer(mmcoDualModuleInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateInputCost> MMCODualInterface::getQuadraticInputCost(const std::string& taskFile)
{
  ocs2::matrix_t R = ocs2::matrix_t::Zero(mmcoDualModuleInfo_.inputDim, mmcoDualModuleInfo_.inputDim);
  const int baseInputDim = mmcoDualModuleInfo_.inputDim - mmcoDualModuleInfo_.leftArmDim - mmcoDualModuleInfo_.rightArmDim;
  const int leftArmStateDim = mmcoDualModuleInfo_.leftArmDim;
  const int rightArmStateDim = mmcoDualModuleInfo_.rightArmDim;

  // arm base DOFs input costs
  if (baseInputDim > 0) {
    ocs2::matrix_t R_base = ocs2::matrix_t::Zero(baseInputDim, baseInputDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R.base", R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  // left arm joints DOFs input costs
  ocs2::matrix_t R_leftArm = ocs2::matrix_t::Zero(leftArmStateDim, leftArmStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R.leftarm", R_leftArm);
  // right arm joints 
  ocs2::matrix_t R_rightArm = ocs2::matrix_t::Zero(rightArmStateDim, rightArmStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R.rightarm", R_rightArm);
  // add in R
  const int armDim = leftArmStateDim + rightArmStateDim;
  ocs2::matrix_t R_temp = ocs2::matrix_t::Zero(armDim,armDim);
  R_temp.topLeftCorner(leftArmStateDim,leftArmStateDim) = R_leftArm;
  R_temp.bottomRightCorner(rightArmStateDim,rightArmStateDim) = R_rightArm;
  R.bottomRightCorner(armDim, armDim) = R_temp;

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::make_unique<mmco_dual_interface::QuadraticInputCost>(std::move(R), mmcoDualModuleInfo_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateInputCost> MMCODualInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  ocs2::loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  const int baseStateDim = mmcoDualModuleInfo_.stateDim - mmcoDualModuleInfo_.leftArmDim -mmcoDualModuleInfo_.rightArmDim ;
  const int leftArmStateDim = mmcoDualModuleInfo_.leftArmDim;
  const int rightArmStateDim = mmcoDualModuleInfo_.rightArmDim;
  const int armStateDim = leftArmStateDim + rightArmStateDim;
  const int baseInputDim = mmcoDualModuleInfo_.inputDim - mmcoDualModuleInfo_.rightArmDim - mmcoDualModuleInfo_.leftArmDim;
  const int leftArmInputDim = mmcoDualModuleInfo_.leftArmDim;
  const int rightArmInputDim = mmcoDualModuleInfo_.rightArmDim;
  const int armInputDim = leftArmInputDim + rightArmInputDim;
  const auto& model = pinocchioInterface.getModel();

  // Load position limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) {
    ocs2::scalar_t muPositionLimits = 1e-2;
    ocs2::scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF
    const ocs2::vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
    const ocs2::vector_t upperBound = model.upperPositionLimit.tail(armStateDim);

    std::cerr << "\n #### JointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    ocs2::loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    ocs2::loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i) {
      ocs2::StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = baseStateDim + i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new ocs2::RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }
  // load velocity limits
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    vector_t lowerBound = vector_t::Zero(mmcoDualModuleInfo_.inputDim);
    vector_t upperBound = vector_t::Zero(mmcoDualModuleInfo_.inputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    // Base DOFs velocity limits
    if (baseInputDim > 0) {
      vector_t lowerBoundBase = vector_t::Zero(baseInputDim);
      vector_t upperBoundBase = vector_t::Zero(baseInputDim);
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.lowerBound.base",
                                lowerBoundBase);
      loadData::loadEigenMatrix(taskFile,
                                "jointVelocityLimits.upperBound.base",
                                upperBoundBase);
      lowerBound.head(baseInputDim) = lowerBoundBase;
      upperBound.head(baseInputDim) = upperBoundBase;
    }

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(mmcoDualModuleInfo_.inputDim);
    std::cerr << " #### =============================FOR======================================\n";
    for (int i = 0; i < mmcoDualModuleInfo_.inputDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->initializeOffset(0.0, vector_t::Zero(mmcoDualModuleInfo_.stateDim), vector_t::Zero(mmcoDualModuleInfo_.inputDim));
  return boxConstraints;
}

std::unique_ptr<ocs2::StateCost> MMCODualInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries,const std::string endEffectorName)
{
  ocs2::scalar_t muPosition = 1.0;
  ocs2::scalar_t muOrientation = 1.0;
  const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  ocs2::loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  ocs2::loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### liushuo329===================================================\n";

  if (referenceManagerPtr_== nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
  }
  std::cerr << " #### liushuo line334===============================================\n";
  std::unique_ptr<StateConstraint> constraint;
  std::cerr << usePreComputation << " #### liushuo line336========\n";
  if (usePreComputation) {
    std::cerr << " #### ==========================MMCODualPinocchioMapping=======================\n";
    MMCODualPinocchioMapping pinocchioMapping(mmcoDualModuleInfo_);
    std::cerr << " #### ==========================PinocchioEndEffectorKinematics=======================\n";
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {endEffectorName});
    std::cerr << " #### ==========================EndEffectorConstraint=======================\n";
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_,endEffectorName));
    std::cerr << " #### ==========================good=======================\n";
  } else {
    
    
    #ifdef mode
    MMCODualPinocchioMappingCppAd pinocchioMappingCppAd(mmcoDualModuleInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {endEffectorName},
                                                     mmcoDualModuleInfo_.stateDim-3, mmcoDualModuleInfo_.inputDim-3,
                                                     prefix+"_end_effector_kinematics", libraryFolder, recompileLibraries, false);
    #else
    MMCODualWBPinocchioMappingCppAd pinocchioMappingCppAd(mmcoDualModuleInfo_);
    std::cerr <<"状态维度："<< mmcoDualModuleInfo_.stateDim<<"line353"<<"输入维度："<<mmcoDualModuleInfo_.inputDim<<"========\n";
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {endEffectorName},
                                                     mmcoDualModuleInfo_.stateDim, mmcoDualModuleInfo_.inputDim,
                                                     prefix+"_end_effector_kinematics", libraryFolder, recompileLibraries, false);
    std::cerr << "### liushuo line357=======\n";
    #endif
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_,endEffectorName));
    std::cerr << "### liushuo line359=======\n";
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });
  std::cerr << "完成line357";
  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

std::unique_ptr<ocs2::StateCost> MMCODualInterface::getBaseConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries)
{
  ocs2::scalar_t muPosition= 1.0;
  ocs2::scalar_t muOrientation = 1.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  ocs2::loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  ocs2::loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_== nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
  }


  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    MMCODualWBPinocchioMapping pinocchioMapping(mmcoDualModuleInfo_);
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping,{"base_link"});
    constraint.reset(new BaseConstraint(eeKinematics, *referenceManagerPtr_));
  } else {
    MMCODualWBPinocchioMappingCppAd pinocchioMappingCppAd(mmcoDualModuleInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {"base_link"},
                                                     mmcoDualModuleInfo_.stateDim, mmcoDualModuleInfo_.inputDim,
                                                     prefix+"_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new BaseConstraint(eeKinematics, *referenceManagerPtr_));
  }

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  std::cerr << "完成底盘软约束\n";
  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
  
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateCost> MMCODualInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, const std::string& urdfFile,
                                                                                  const std::string& prefix, bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    constraint = std::make_unique<MMCODualSelfCollisionConstraint>(MMCODualPinocchioMapping(mmcoDualModuleInfo_),
                                                                            std::move(geometryInterface), minimumDistance);
  } else {
    constraint = std::make_unique<SelfCollisionConstraintCppAd>(
        pinocchioInterface, MMCODualPinocchioMapping(mmcoDualModuleInfo_), std::move(geometryInterface), minimumDistance,
        "self_collision", libraryFolder, recompileLibraries, false);
  }

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}


}