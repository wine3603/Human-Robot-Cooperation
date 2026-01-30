#include<baichuan_arm_dynamics_models/elfin/elfin5_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

Elfin5Pose1DynamicsModel::Elfin5Pose1DynamicsModel() : robot_dof_(6), params_total_amount_(78),
                                                       basic_params_amount_(52), gravity_basic_params_amount_(10)
{
  int array_tmp[52]={3, 11, 12, 13, 14, 15, 17, 18, 19, 20, 24, 25, 26, 27, 28,
                     29, 30, 32, 34, 36, 37, 38, 39, 40, 41, 42, 43, 45, 47, 49,
                     50, 51, 52, 53, 54, 55, 56, 58, 60, 62, 63, 64, 65, 66, 67,
                     69, 70, 71, 72, 75, 76, 77};
  basic_params_index_=std::vector<size_t>(array_tmp, array_tmp+52);

  int gravity_array_tmp[10]={19, 20, 32, 34, 45, 47, 58, 60, 71, 72};
  gravity_basic_params_index_=std::vector<size_t>(gravity_array_tmp, gravity_array_tmp+10);

  int fc_array_tmp[6]={12, 25, 38, 51, 64, 77};
  coulomb_friction_params_index_=std::vector<size_t>(fc_array_tmp, fc_array_tmp+6);

  int fv_array_tmp[6]={11, 24, 37, 50, 63, 76};
  viscous_friction_params_index_=std::vector<size_t>(fv_array_tmp, fv_array_tmp+6);

  double gravity_tmp[3]={0.0, 0.0, -9.788};
  gravity_vector_=std::vector<double>(gravity_tmp, gravity_tmp+3);
}

Elfin5Pose1DynamicsModel::~Elfin5Pose1DynamicsModel()
{

}

int Elfin5Pose1DynamicsModel::get_robot_dof()
{
  return robot_dof_;
}

int Elfin5Pose1DynamicsModel::get_params_total_amount()
{
  return params_total_amount_;
}

int Elfin5Pose1DynamicsModel::get_basic_params_amount()
{
  return basic_params_amount_;
}

std::vector<size_t> Elfin5Pose1DynamicsModel::get_basic_params_index()
{
  return basic_params_index_;
}

int Elfin5Pose1DynamicsModel::get_gravity_basic_params_amount()
{
  return gravity_basic_params_amount_;
}

std::vector<size_t> Elfin5Pose1DynamicsModel::get_gravity_basic_params_index()
{
  return gravity_basic_params_index_;
}

std::vector<size_t> Elfin5Pose1DynamicsModel::get_coulomb_friction_params_index()
{
  return coulomb_friction_params_index_;
}

std::vector<size_t> Elfin5Pose1DynamicsModel::get_viscous_friction_params_index()
{
  return viscous_friction_params_index_;
}

std::vector<double> Elfin5Pose1DynamicsModel::get_gravity_vector()
{
  return gravity_vector_;
}

} // end namespace robot_dynamics
}
} // end namespace moying
