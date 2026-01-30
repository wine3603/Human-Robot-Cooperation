#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "pr2_wholebody_interface/wholebody_module_info.h"



namespace pr2_wholebody_interface{
using namespace ocs2;
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const std::vector<std::string>& JointNames);
ocs2::PinocchioInterface createPinocchioInterfaceWithoutBase(const std::string& robotUrdfPath, const std::vector<std::string>& JointNames);
PR2WBCModuleInfo createPR2WBCModuleInfo(const PinocchioInterface& interface,const std::string&baseFrame,
                                            const std::string& eleFrame, const std::string& ereFrame);

}//pr2_wholebody_interface
