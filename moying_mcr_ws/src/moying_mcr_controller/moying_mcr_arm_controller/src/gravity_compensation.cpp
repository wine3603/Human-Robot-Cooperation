#include "velocity_interface_controller/gravity_compensation.h"
#include <cmath>

namespace velocity_interface_controller{
using namespace std;
void grav_compen(std::vector<double> &jnt_pos_,std::vector<double> &jnt_vel_,std::vector<double> &T_cmd_,std::vector<double> &T_out_)
{
  T_out_.resize(6);
  jnt_pos_[0]= joint1_dir* (jnt_pos_[0]+joint1_offset);    //第一个轴偏移90度
  jnt_pos_[1]= joint2_dir* (jnt_pos_[1]+joint2_offset);    //第一个轴偏移90度
  jnt_pos_[2]= joint3_dir* (jnt_pos_[2]+joint3_offset);
  jnt_pos_[3]= joint4_dir* (jnt_pos_[3]+joint4_offset);
  jnt_pos_[4]= joint5_dir* (jnt_pos_[4]+joint5_offset);
  jnt_pos_[5]= joint6_dir* (jnt_pos_[5]+joint6_offset);
  double c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6;
  c1=cos(jnt_pos_[0]); s1=sin(jnt_pos_[0]); c2=cos(jnt_pos_[1]); s2=sin(jnt_pos_[1]); c3=cos(jnt_pos_[2]); s3=sin(jnt_pos_[2]);
  c4=cos(jnt_pos_[3]); s4=sin(jnt_pos_[3]); c5=cos(jnt_pos_[4]); s5=sin(jnt_pos_[4]);
  double s23=c2*s3+c3*s2;
  double ox5=- s5*(s1*s4 - c4*(c1*s2*s3 - c1*c2*c3)) - c5*(c1*c2*s3 + c1*c3*s2);
  double oy5= s5*(c1*s4 + c4*(s1*s2*s3 - c2*c3*s1)) - c5*(c2*s1*s3 + c3*s1*s2);
  double ax5=c4*s1 + s4*(c1*s2*s3 - c1*c2*c3);
  double ay5= s4*(s1*s2*s3 - c2*c3*s1) - c1*c4;
  std::vector<double> F_c(6,0);
  T_out_[0]=F_c[0]+T_cmd_.at(0);
  T_out_[1]=T_Gian*(-mg3*(d3*s23-2*d2*c2)+mg2*d2*c2) +F_c[1]+T_cmd_.at(1);
  T_out_[2]=T_Gian*mg3*d3*s23+F_c[2]+T_cmd_.at(2);
  T_out_[3]=F_c[3]+T_cmd_.at(3);
  T_out_[4]=T_Gian*((-oy5*ax5+ox5*ay5)*mg5*d5)+F_c[4]+T_cmd_.at(4);
  T_out_[5]=F_c[5]+T_cmd_.at(5);
  double ax4= - c1*c2*s3 - c1*c3*s2;
  double ay4= - c2*s1*s3 - c3*s1*s2;
  T_out_[3]=T_Gian*((oy5*ax4-ox5*ay4)*mg5_2*d5)+F_c[3]+T_cmd_.at(3);
}
std::vector<double> velPIDControl(std::vector<double>& jntPos,std::vector<double>& jntVel,std::vector<double> &refVel)
{
  static std::vector<double> lastJntVel = jntVel;
  std::vector<pid_t> pidparam(6);
  pidparam[0].kp = 300.0;
  pidparam[0].kd = 0.0;

  pidparam[1].kp = 400.0;
  pidparam[1].kd = 0.05;

  pidparam[2].kp = 250.0;
  pidparam[2].kd = 0.01;

  pidparam[3].kp = 100.0;
  pidparam[3].kd = 0.05;

  pidparam[4].kp = 120.0;
  pidparam[4].kd = 0.05;

  pidparam[5].kp = 80.0;
  pidparam[5].kd = 0.05;

  std::vector<double> pd_value(jntPos.size());
  for(size_t i=0;i<jntPos.size();i++)
  {
    pd_value[i] = pidparam[i].kp * (refVel[i] - jntVel[i]) + pidparam[i].kd * (jntVel[i] - lastJntVel[i])/0.001;
  }
  lastJntVel = jntVel;
  return pd_value;

}
}//namespace mmco_controller