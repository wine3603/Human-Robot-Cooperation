#pragma once

// C/C++
#include <string>
#include <vector>
namespace pr2_wholebody_interface{
struct PR2WBCModuleInfo{
  size_t stateDim;
  size_t inputDim;
  size_t torsoLiftDim;
  size_t leftArmDim;
  size_t rightArmDim;
  size_t baseDim;

  std::string baseFrame;
  std::string eleFrame;
  std::string ereFrame;
  std::vector<std::string> dofNames;
};
}