#pragma once

// C/C++
#include <string>
#include <vector>
namespace mmco_dual_interface{
struct MMCODualModuleInfo{
  size_t stateDim;
  size_t inputDim;
  size_t leftArmDim;
  size_t rightArmDim;

  std::string baseFrame;
  std::string eleFrame;
  std::string ereFrame;
  std::vector<std::string> dofNames;
};
}