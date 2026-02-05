#include "mmco_dual_interface/factory_functions.h"

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include "mmco_dual_interface/mmco_dual_module_info.h"

namespace mmco_dual_interface {

ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const std::vector<std::string>& jointNames)
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for(auto jn: jointNames){
    std::cerr << "joint name: " << jn << std::endl;
  }
  for (joint_pair_t& jointPair : newModel->joints_) {
    std::cerr << "model joint  name: " << jointPair.first<< std::endl;
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
      std::cerr << "Found joint name: " << jointPair.first << std::endl;
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  pinocchio::JointModelComposite jointComposite(3);
  jointComposite.addJoint(pinocchio::JointModelPX());
  jointComposite.addJoint(pinocchio::JointModelPY());
  jointComposite.addJoint(pinocchio::JointModelRZ());
  // return pinocchio interface
  return ocs2::getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
}
ocs2::PinocchioInterface createPinocchioInterfaceWithoutBase(const std::string& robotUrdfPath, const std::vector<std::string>& jointNames)
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for(auto jn: jointNames){
    std::cerr << "joint name: " << jn << std::endl;
  }
  for (joint_pair_t& jointPair : newModel->joints_) {
    std::cerr << "model joint  name: " << jointPair.first<< std::endl;
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
      std::cerr << "Found joint name: " << jointPair.first << std::endl;
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  return getPinocchioInterfaceFromUrdfModel(newModel);
}
MMCODualModuleInfo createMMCODualModuleInfo(const ocs2::PinocchioInterface& interface, const std::string& baseFrame,
                                            const std::string& eleFrame, const std::string& ereFrame)
{
  const auto& model = interface.getModel();
  MMCODualModuleInfo info;
  info.stateDim = model.nq;
  std::cerr << "\n #### nq:" << info.stateDim << std::endl;
  info.inputDim = info.stateDim;
  std::cerr << "\n #### inputDim:" << info.inputDim << std::endl;
  info.leftArmDim = (info.stateDim-3) / 2;
  std::cerr << "\n #### leftArmDim:" << info.leftArmDim << std::endl;
  info.rightArmDim = (info.stateDim-3) / 2;
  std::cerr << "\n #### rightArmDim:" << info.rightArmDim << std::endl;
  info.baseFrame = baseFrame;
  std::cerr << "\n #### baseFrame:" << info.baseFrame << std::endl;
  info.eleFrame = eleFrame;
  std::cerr << "\n #### eleFrame:" << info.eleFrame << std::endl;
  info.ereFrame = ereFrame;
  std::cerr << "\n #### ereFrame:" << info.ereFrame << std::endl;
  const auto& jointNames = model.names;
  info.dofNames = std::vector<std::string>(jointNames.end() - info.leftArmDim - info.rightArmDim, jointNames.end());
  return info;
}

} //namespace mmco_dual_interfac