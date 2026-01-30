#include <moying_mcr_arm_dynamics_models/dynamics_models.h>
#include <memory>
#include <chrono>
#include <iostream>


using namespace baichuan::arm;

std::shared_ptr<BasicDynamicsModel> dyn_model;
std::vector<double> elfin5_basic_param = {5.5834, 20.6174, 11.3804, -3.93229, 1.11347, 0.544532, -0.85155, -0.241093,
             4.68726, 0.202398, 6.71532, 10.4753, -0.0813052, 0.189768, -0.291738, 0.821854,
             -0.800556, -0.144557, 1.94197, 0.0169887, 4.39409, 4.80435, 0.608595, 0.156044,
             -0.0732623, 0.0953026, -0.462754, 0.0115155, -0.124098, 0.680114, 5.51965, 3.97381,
             -0.456986, 0.0365659, -0.163843, -0.0654413, 0.0572246, 0.0263394, 0.191243, 0.751408,
             3.63408, 2.46535, -0.134029, 0.0722608, -0.00596429, -0.125216, -0.0023432, -0.0204092,
             -1.62148e-05, 0.344182, 3.56147, 3.12429};

std::vector<double> elfin3_basic_param = {-9.5323, -0.494575, 0.217726, 10.8791, 6.97552, 11.1947, -2.01641,
           1.54969, 1.3911, 1.0394, 1.89893, 0.281033, 7.13797, 6.14408, 2.63808, 0.815542,
           2.21467, 1.38974, -0.3248, 0.0666458, 1.75996, 0.724542, 4.01041, 3.30275, 0.69838,
           -0.168122, -0.173828, 0.0744014, 0.408503, 0.00621546, -0.127075, 1.2218, 3.71925,
           2.70396, -0.172553, 0.0422848, -0.0134991, -0.180478, 0.238172, -0.0160892, 0.379865,
           0.820333, 2.89437, 2.21194, 0.152857, 0.0173616, -0.0890912, -0.0366125, 0.00348761,
           0.0175657, 0.00762579, 0.441349, 2.95611, 2.67612};

// std::vector<double> joint_position = {0.0, 1.570796327, 0.0, 0.0, 1.570796327, 0.0};
// std::vector<double> joint_velocity = {0.0, 1.570796327, 0.0, 0.0, 1.570796327, 0.0};
// std::vector<double> joint_acceleration = {0.0, 1.570796327, 0.0, 1.570796327, 0.0, 0.0};


int main(int argc, char const *argv[])
{
  //测试动力学模型
  Eigen::VectorXd eigen_joint_position;
  Eigen::VectorXd eigen_joint_velocity;
  Eigen::VectorXd eigen_joint_acceleration;
  
  // 测试实时性
  // auto time_start = std::chrono::system_clock::now();
  // 5公斤机械臂
  for (size_t i = 0; i < 10; i++)
  {
    dyn_model =  std::make_shared<elfin::Elfin5Pose1DynamicsModel>();
    dyn_model->init(elfin5_basic_param);
    eigen_joint_position = Eigen::MatrixXd::Random(6,1) * M_PI;
    eigen_joint_velocity = Eigen::MatrixXd::Random(6,1) * M_PI;
    eigen_joint_acceleration = Eigen::MatrixXd::Random(6,1) * M_PI;
    std::cout << "joint_position: " << eigen_joint_position.transpose() << std::endl;
    std::cout << "joint_velocity: " << eigen_joint_velocity.transpose() << std::endl;
    std::cout << "joint_acceleration: " << eigen_joint_acceleration.transpose() << std::endl;
  
    std::vector<double> joint_position = std::vector<double>(eigen_joint_position.data(), eigen_joint_position.data()+ eigen_joint_position.size());
    std::vector<double> joint_velocity = std::vector<double>(eigen_joint_velocity.data(), eigen_joint_velocity.data()+ eigen_joint_velocity.size());
    std::vector<double> joint_acceleration = std::vector<double>(eigen_joint_acceleration.data(), eigen_joint_acceleration.data()+ eigen_joint_acceleration.size());
  
    // tau = M+C+G+F 
    Eigen::VectorXd tau =  dyn_model->tau(joint_position,joint_velocity,joint_acceleration);
    // 重力项
    Eigen::VectorXd gravity_term = dyn_model->gravityTerm(position_state);
    // 库伦摩擦
    Eigen::VectorXd coulomb_term = dyn_model->coulombFriction(velocity_state);
    // 粘滞摩擦
    Eigen::VectorXd viscous_term = dyn_model->viscousFriction(velocity_state);
    std::cout << "tau: " <<tau.transpose() << std::endl;
    std::cout << "gravity: " << gravity_term.transpose() << std::endl;
  }

  // 3公斤机械臂侧装

  for (size_t i = 0; i < 10; i++)
  {
    dyn_model =  std::make_shared<elfin::Elfin3Pose2DynamicsModel>();
    dyn_model->init(elfin3_basic_param);
    eigen_joint_position = Eigen::MatrixXd::Random(6,1) * M_PI;
    eigen_joint_velocity = Eigen::MatrixXd::Random(6,1) * M_PI;
    eigen_joint_acceleration = Eigen::MatrixXd::Random(6,1) * M_PI;
    // std::cout << "joint_position: " << eigen_joint_position.transpose() << std::endl;
    // std::cout << "joint_velocity: " << eigen_joint_velocity.transpose() << std::endl;
    // std::cout << "joint_acceleration: " << eigen_joint_acceleration.transpose() << std::endl;
  
    std::vector<double> joint_position = std::vector<double>(eigen_joint_position.data(), eigen_joint_position.data()+ eigen_joint_position.size());
    std::vector<double> joint_velocity = std::vector<double>(eigen_joint_velocity.data(), eigen_joint_velocity.data()+ eigen_joint_velocity.size());
    std::vector<double> joint_acceleration = std::vector<double>(eigen_joint_acceleration.data(), eigen_joint_acceleration.data()+ eigen_joint_acceleration.size());
  
    Eigen::VectorXd gravity =  dyn_model->gravityTerm(joint_position);
    Eigen::VectorXd tau =  dyn_model->tau(joint_position,joint_velocity,joint_acceleration);
    // std::cout << "tau: " <<tau.transpose() << std::endl;
    // std::cout << "gravity: " << gravity.transpose() << std::endl;
  }
  return 0;
}

