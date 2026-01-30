#include "moying_mcr_hardware_zombie/arm_ethercat_driver.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <moying_mcr_arm_dynamics_models/dynamics_models.h>
#include "yaml-cpp/yaml.h"
#include <rclcpp/node.hpp>
#include <string>
#include <vector>
#include <cmath>




namespace moying_mcr_hardware_zombie
{

typedef struct
{
  std::string axis1;
  std::string axis2;
}dulinterface_t;

ArmEthercatDriver::ArmEthercatDriver(std::shared_ptr<elfin_ethercat_driver_zombie::EtherCatManager> ecat_manager,
    const rclcpp::Node::SharedPtr& node,YAML::Node& config)
{
  em = ecat_manager.get();
  n_ = rclcpp::Node::make_shared(config["if_name"].as<std::string>()+"elfin_hw");
  in_effort_mode_.resize(3);
  in_effort_mode_[0] = false;
  in_effort_mode_[1] = false;
  in_effort_mode_[2] = false;
  elfin_driver_names_.resize(1);
  if(config["elfin_ethercat_drivers"])
    elfin_driver_names_[0] = config["elfin_ethercat_drivers"].as<std::string>();
  else
    elfin_driver_names_[0] = "elfin";
  RCLCPP_INFO(rclcpp::get_logger("ArmEthercatDriver"),"Init ElfinEtherCATDriver driver:%s",elfin_driver_names_[0].c_str());
  ethercat_drivers_.resize(elfin_driver_names_.size());
  for(unsigned int i=0;i<elfin_driver_names_.size();i++)
  {
    ethercat_drivers_[i] = new elfin_ethercat_driver_zombie::ElfinEtherCATDriver(em,elfin_driver_names_[i],n_,config);
  }
  module_infos.clear();
  RCLCPP_INFO(rclcpp::get_logger("ArmEthercatDriver"),"ElfinEtherCATDriver driver client:%d",ethercat_drivers_[0]->getEtherCATClientNumber());

  for (size_t i =0;i<ethercat_drivers_.size();i++)
  {
    for(size_t j =0;j<ethercat_drivers_[i]->getEtherCATClientNumber();j++)
    {
      ModuleInfo module_info_tmp;
      module_info_tmp.client_ptr = ethercat_drivers_[i]->getEtherCATClientPtr(j);
      module_info_tmp.axis1.name = ethercat_drivers_[i]->getJointName(2*j);
      module_info_tmp.axis1.reduction_ratio = ethercat_drivers_[i]->getReductionRatio(2*j);
      module_info_tmp.axis1.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j);
      module_info_tmp.axis1.count_zero = ethercat_drivers_[i]->getCountZero(2*j);
      module_info_tmp.axis1.axis_torque_factor = ethercat_drivers_[i]->getAxisTorqueFactor(2*j);

      module_info_tmp.axis2.name = ethercat_drivers_[i]->getJointName(2*j+1);
      module_info_tmp.axis2.reduction_ratio = ethercat_drivers_[i]->getReductionRatio(2*j+1);
      module_info_tmp.axis2.axis_position_factor=ethercat_drivers_[i]->getAxisPositionFactor(2*j+1);
      module_info_tmp.axis2.count_zero =ethercat_drivers_[i]->getCountZero(2*j+1);
      module_info_tmp.axis2.axis_torque_factor = ethercat_drivers_[i]->getAxisTorqueFactor(2*j+1);
      module_infos.push_back(module_info_tmp);
    }
  }

  for(size_t i =0;i<module_infos.size();i++)
  {
    module_infos[i].axis1.count_rad_factor = module_infos[i].axis1.reduction_ratio*module_infos[i].axis1.axis_position_factor/(2*M_PI);
    module_infos[i].axis1.count_rad_per_s_factor = module_infos[i].axis1.count_rad_factor/750.3;
    module_infos[i].axis1.count_Nm_factor = module_infos[i].axis1.axis_torque_factor / module_infos[i].axis1.reduction_ratio;

    module_infos[i].axis2.count_rad_factor = module_infos[i].axis2.reduction_ratio*module_infos[i].axis2.axis_position_factor/(2*M_PI);
    module_infos[i].axis2.count_rad_per_s_factor = module_infos[i].axis2.count_rad_factor/750.3;
    module_infos[i].axis2.count_Nm_factor = module_infos[i].axis2.axis_torque_factor / module_infos[i].axis2.reduction_ratio;
  }
  

  for(size_t i=0; i<module_infos.size(); i++)
  {
      int32_t pos_count1=module_infos[i].client_ptr->getAxis1PosCnt();
      double position_tmp_1=(pos_count1-module_infos[i].axis1.count_zero)/module_infos[i].axis1.count_rad_factor;
      if(position_tmp_1>=M_PI)
      {
          module_infos[i].axis1.count_zero+=module_infos[i].axis1.count_rad_factor*2*M_PI;
      }
      else if(position_tmp_1<-1*M_PI)
      {
          module_infos[i].axis1.count_zero-=module_infos[i].axis1.count_rad_factor*2*M_PI;
      }
      module_infos[i].axis1.position=-1*(pos_count1-module_infos[i].axis1.count_zero)/module_infos[i].axis1.count_rad_factor;

      int32_t pos_count2=module_infos[i].client_ptr->getAxis2PosCnt();
      double position_tmp_2=(pos_count2-module_infos[i].axis2.count_zero)/module_infos[i].axis2.count_rad_factor;
      if(position_tmp_2>=M_PI)
      {
          module_infos[i].axis2.count_zero+=module_infos[i].axis2.count_rad_factor*2*M_PI;
      }
      else if(position_tmp_2<-1*M_PI)
      {
          module_infos[i].axis2.count_zero-=module_infos[i].axis2.count_rad_factor*2*M_PI;
      }
      module_infos[i].axis2.position=-1*(pos_count2-module_infos[i].axis2.count_zero)/module_infos[i].axis2.count_rad_factor;
  }

  pre_switch_flags_.resize(module_infos.size());
  for(unsigned int i=0;i<pre_switch_flags_.size();i++)
  {
    pre_switch_flags_[i] = false;
  }

  pre_switch_mutex_ptrs_.resize(module_infos.size());
  for(unsigned int i=0;i<pre_switch_mutex_ptrs_.size();i++)
  {
    pre_switch_mutex_ptrs_[i] = boost::shared_ptr<boost::mutex>(new boost::mutex);
  }

}
ArmEthercatDriver::~ArmEthercatDriver()
{
  ;
}
hardware_interface::return_type ArmEthercatDriver::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm"),"Switch");
  std::list<std::string>::const_iterator iter;
  std::vector<dulinterface_t> index_dualinterface(module_infos.size());
  for(auto& key : start_interfaces)
  {
    for(std::size_t i=0;i<module_infos.size();i++)
    {
      if(key == module_infos[i].axis1.name + "/" + "position")
      {
        index_dualinterface[i].axis1 = "position";
      }
      if(key == module_infos[i].axis1.name + "/" + "velocity")
      {
        index_dualinterface[i].axis1 = "velocity";
      }
      if(key == module_infos[i].axis1.name + "/" + "effort")
      {
        index_dualinterface[i].axis1 = "effort";
      }
      
      if(key == module_infos[i].axis2.name + "/" + "position")
      {
        index_dualinterface[i].axis2 = "position";
      }
      if(key == module_infos[i].axis2.name + "/" + "velocity")
      {
        index_dualinterface[i].axis2 = "velocity";
      }
      if(key == module_infos[i].axis2.name + "/" + "effort")
      {
        index_dualinterface[i].axis2 = "effort";
      }
    }
  }
  for(int i=0;i<module_infos.size();i++)
  {
    if(index_dualinterface[i].axis1 == index_dualinterface[i].axis2)
    {
      if(index_dualinterface[i].axis1 == "position")
      {
        if(in_effort_mode_[i]==false){
        std::vector ki={i};
        if(module_infos[i].client_ptr->isEnabled())
          if(!setGroupPosMode(ki))
            return hardware_interface::return_type::ERROR;
          in_effort_mode_[i] = true;        
        }
      }
      if(index_dualinterface[i].axis1 == "effort")
      {
        if(in_effort_mode_[i]==false){
          std::vector ki={i};
          if(module_infos[i].client_ptr->isEnabled()){
            RCLCPP_INFO(rclcpp::get_logger("Arm"),"Trq Switch well");
            if(!setGroupTrqMode(ki))
              return hardware_interface::return_type::ERROR;
          }
          in_effort_mode_[i] = true;        
        }
      }
    }
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type ArmEthercatDriver::perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&)
{
  std::shared_ptr<baichuan::arm::BasicDynamicsModel> dyn_model;
  std::vector<double> elfin3_basic_param = {-9.5323, -0.494575, 0.217726, 10.8791, 6.97552, 11.1947, -2.01641,
           1.54969, 1.3911, 1.0394, 1.89893, 0.281033, 7.13797, 6.14408, 2.63808, 0.815542,
           2.21467, 1.38974, -0.3248, 0.0666458, 1.75996, 0.724542, 4.01041, 3.30275, 0.69838,
           -0.168122, -0.173828, 0.0744014, 0.408503, 0.00621546, -0.127075, 1.2218, 3.71925,
           2.70396, -0.172553, 0.0422848, -0.0134991, -0.180478, 0.238172, -0.0160892, 0.379865,
           0.820333, 2.89437, 2.21194, 0.152857, 0.0173616, -0.0890912, -0.0366125, 0.00348761,
           0.0175657, 0.00762579, 0.441349, 2.95611, 2.67612};
  dyn_model = std::make_shared<baichuan::arm::elfin::Elfin3Pose2DynamicsModel>();
  dyn_model->init(elfin3_basic_param);
  std::vector<double> joint_position = {module_infos[0].axis1.position,module_infos[0].axis2.position,
    module_infos[1].axis1.position,module_infos[1].axis2.position,
    module_infos[2].axis1.position,module_infos[2].axis2.position};
  Eigen::VectorXd gravity =  dyn_model->gravityTerm(joint_position);

  for(size_t i=0; i<pre_switch_flags_.size(); i++)
  {
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
      if(pre_switch_flags_[i])
      {
          module_infos[i].axis1.velocity_cmd=0;
          module_infos[i].axis1.vel_ff_cmd=0;
          module_infos[i].axis1.effort_cmd=gravity(2*i);

          module_infos[i].axis2.velocity_cmd=0;
          module_infos[i].axis2.vel_ff_cmd=0;
          module_infos[i].axis2.effort_cmd=gravity(2*i+1);

          pre_switch_flags_[i]=false;
      }
      pre_switch_flags_lock.unlock();
  }
  return hardware_interface::return_type::OK;

}

bool ArmEthercatDriver::setGroupTrqMode(const std::vector<int> &module_no)
{
  for(int j=0; j<module_no.size(); j++)
  {
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
      pre_switch_flags_[module_no[j]]=true;
      pre_switch_flags_lock.unlock();
  }
  usleep(5000);
  for(int j=0; j<module_no.size(); j++)
  {
    module_infos[module_no[j]].client_ptr->setAxis1TrqCnt(0x0);
    module_infos[module_no[j]].client_ptr->setAxis2TrqCnt(0x0);
  }


  for(int j=0; j<module_no.size(); j++)
  {
      module_infos[module_no[j]].client_ptr->setTrqMode();
  }
  usleep(10000);

  bool set_trq_success=true;
  for(int j=0; j<module_no.size(); j++)
  {
    if(!module_infos[module_no[j]].client_ptr->inTrqMode())
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArmEthercatDriver"),"Failed to set trq mode");
      set_trq_success=false;
      break;
    }
  }
  if(!set_trq_success)
  {
    for(int j=0; j<module_no.size(); j++)
    {
        module_infos[module_no[j]].client_ptr->setPosMode();
    }
    usleep(10000);
    for(int j=0; j<module_no.size(); j++)
    {
      if(!module_infos[module_no[j]].client_ptr->inPosMode())
      {
        RCLCPP_ERROR(rclcpp::get_logger("ArmEthercatDriver"),"Failed to set position mode,too");
      }
    }

    for(int j=0; j<module_no.size(); j++)
    {
      boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
      pre_switch_flags_[module_no[j]]=false;
      pre_switch_flags_lock.unlock();
    }
    return false;
  }
  return true;
}

bool ArmEthercatDriver::setGroupPosMode(const std::vector<int> &module_no)
{
  for(int j=0; j<module_no.size(); j++)
  {
    boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[j]]);
    pre_switch_flags_[module_no[j]]=true;
    pre_switch_flags_lock.unlock();
  }

  usleep(5000);

  for(int j=0; j<module_no.size(); j++)
  {
    module_infos[module_no[j]].client_ptr->setAxis1VelFFCnt(0x0);
    module_infos[module_no[j]].client_ptr->setAxis2VelFFCnt(0x0);

    module_infos[module_no[j]].client_ptr->setAxis1TrqCnt(0x0);
    module_infos[module_no[j]].client_ptr->setAxis2TrqCnt(0x0);
  }

  for(int j=0; j<module_no.size(); j++)
  {
    module_infos[module_no[j]].client_ptr->setPosMode();
  }

  usleep(10000);

  for(int j=0; j<module_no.size(); j++)
  {
    if(!module_infos[module_no[j]].client_ptr->inPosMode())
    {
      RCLCPP_ERROR(rclcpp::get_logger("ArmEthercatDriver"),"setGroupPosMode failed");
      for(int k=0; k<module_no.size(); k++)
      {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
        pre_switch_flags_[module_no[k]]=false;
        pre_switch_flags_lock.unlock();
      }

      return false;
    }
  }

  return true;
}


}// namespace moying_mcr_hardware