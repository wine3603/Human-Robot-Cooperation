#include "velocity_interface_controller/pinocchio_gravity_compensation.h"

#include <Eigen/Dense>

namespace velocity_interface_controller{
GravityCompensation::GravityCompensation(std::string urdf_file_path)
{
    pinocchio::urdf::buildModel(urdf_file_path,*model_);
    data_ = std::make_shared<pinocchio::Data>(*model_);
}
Eigen::VectorXd GravityCompensation::getCompensation(Eigen::VectorXd q)
{
    pinocchio::crba(*model_,*data_,q);
    Eigen::VectorXd tau_gravity = pinocchio::rnea(*model_,*data_,q,Eigen::VectorXd::Zero(model_->nv),Eigen::VectorXd::Zero(model_->nv));
    tau_gravity = -tau_gravity;
    return tau_gravity;
}
}//velocity_interface_controller