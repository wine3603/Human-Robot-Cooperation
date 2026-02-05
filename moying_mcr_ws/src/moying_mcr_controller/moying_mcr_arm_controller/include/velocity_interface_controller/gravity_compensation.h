#pragma onces
#include <vector>
#include <cmath>
#define pi   3.14159
#define joint1_offset -pi/2
#define joint2_offset  pi/2
#define joint3_offset -pi/2
#define joint4_offset 0
#define joint5_offset 0
#define joint6_offset 0

#define joint1_dir     1
#define joint2_dir     -1
#define joint3_dir     1
#define joint4_dir     1
#define joint5_dir     1
#define joint6_dir     1

#define d5    0.2
#define mg5   7
#define mg5_2 4
#define d3  0.21
#define mg3 22
#define d2 0.19
#define mg2 18
#define T_Gian 5
#define ctrl_prd 0.005
#define static_vel 0.01
namespace velocity_interface_controller{
struct pid_t
{
  double kp;
  double kd;
  double last_vel;
};
void grav_compen(std::vector<double> &jnt_pos_,std::vector<double> &jnt_vel_,std::vector<double> &T_cmd_,std::vector<double> &T_out_);
std::vector<double> velPIDControl(std::vector<double>& jntPos,std::vector<double>& jntVel,std::vector<double> &refVel);

} //namespace