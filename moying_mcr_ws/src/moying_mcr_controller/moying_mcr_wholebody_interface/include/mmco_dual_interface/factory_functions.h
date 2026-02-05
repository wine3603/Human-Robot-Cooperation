#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "mmco_dual_interface/mmco_dual_module_info.h"



namespace mmco_dual_interface{
using namespace ocs2;
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const std::vector<std::string>& JointNames);
ocs2::PinocchioInterface createPinocchioInterfaceWithoutBase(const std::string& robotUrdfPath, const std::vector<std::string>& JointNames);
MMCODualModuleInfo createMMCODualModuleInfo(const PinocchioInterface& interface,const std::string&baseFrame,
                                            const std::string& eleFrame, const std::string& ereFrame);

}//mmco_dual_interface
