#pragma once
#include <Eigen/Dense>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/parsers/urdf.hpp"
namespace velocity_interface_controller{
class GravityCompensation
{
public:
    GravityCompensation(std::string urdf_file_path);
    Eigen::VectorXd getCompensation(Eigen::VectorXd q);
private:
    std::string urdf_file_path_;
    std::shared_ptr<pinocchio::Model> model_;
    std::shared_ptr<pinocchio::Data> data_;
};
}//velocity_interface_controller