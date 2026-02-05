#include<baichuan_arm_dynamics_models/elfin/elfin3_pose2_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin3Pose2DynamicsModel::gravityTerm( double* g_out, const double* parms, const double* q )
{
  double x0 = -9.788*cos(q[0]);
  double x1 = 9.788*sin(q[0]);
  double x2 = cos(q[1]);
  double x3 = x0*x2;
  double x4 = cos(q[2]);
  double x5 = sin(q[2]);
  double x6 = -x5;
  double x7 = sin(q[1]);
  double x8 = -x0*x7;
  double x9 = x3*x4 + x6*x8;
  double x10 = cos(q[3]);
  double x11 = sin(q[3]);
  double x12 = x1*x11 + x10*x9;
  double x13 = sin(q[4]);
  double x14 = cos(q[4]);
  double x15 = x3*x5 + x4*x8;
  double x16 = -x15;
  double x17 = -x16;
  double x18 = x12*x13 + x14*x17;
  double x19 = cos(q[5]);
  double x20 = -x11;
  double x21 = x1*x10 + x20*x9;
  double x22 = x12*x14 + x13*x16;
  double x23 = sin(q[5]);
  double x24 = -x23;
  double x25 = x19*x21 + x22*x24;
  double x26 = parms[72]*x18 - parms[73]*x25;
  double x27 = -x21;
  double x28 = x19*x22 + x21*x23;
  double x29 = -x18;
  double x30 = parms[71]*x29 + parms[73]*x28;
  double x31 = parms[74]*x28;
  double x32 = -0.1975*x23;
  double x33 = parms[74]*x25;
  double x34 = x19*x33;
  double x35 = parms[59]*x18 + parms[60]*x27 + x19*x26 + x24*x30 + x31*x32 - 0.1975*x34;
  double x36 = -x14;
  double x37 = parms[71]*x25 - parms[72]*x28;
  double x38 = parms[58]*x21 - parms[59]*x22 + x37;
  double x39 = -parms[45]*x27 - parms[47]*x12 - x13*x35 - x36*x38;
  double x40 = parms[32]*x1 - parms[33]*x9 + x39;
  double x41 = parms[46]*x21 + parms[47]*x17 + x13*x38 + x14*x35;
  double x42 = -x1;
  double x43 = x19*x31;
  double x44 = parms[58]*x29 + parms[60]*x22 + x19*x30 + x23*x26 + x32*x33 + 0.1975*x43;
  double x45 = parms[45]*x16 - parms[46]*x12 + x44;
  double x46 = parms[61]*x22 + x24*x33 + x43;
  double x47 = parms[61]*x18 + parms[74]*x18;
  double x48 = parms[48]*x12 + x13*x47 + x14*x46;
  double x49 = -0.324*x11;
  double x50 = parms[48]*x21 + parms[61]*x21 + x23*x31 + x34;
  double x51 = x10*x50;
  double x52 = parms[33]*x15 + parms[34]*x42 + x10*x41 + x20*x45 + x48*x49 - 0.324*x51;
  double x53 = x10*x48;
  double x54 = parms[32]*x16 + parms[34]*x9 + x10*x45 + x11*x41 + x49*x50 + 0.324*x53;
//
  g_out[0] = parms[6]*x1 + parms[8]*x0 + x2*(parms[20]*x1 - parms[21]*x3 + x4*x40 + x52*x6) + x7*(-0.266*((x2)*(x2)) - 0.266*((x7)*(x7)))*(parms[22]*x1 + parms[35]*x1 + x11*x48 + x51) + x7*(parms[19]*x42 + parms[21]*x8 + x4*x52 + x40*x5);
  g_out[1] = -parms[19]*x3 + parms[20]*x8 - 0.266*parms[22]*x3 - 0.266*x4*(parms[35]*x9 + x20*x50 + x53) - 0.266*x5*(parms[35]*x15 - parms[48]*x16 - x13*x46 - x36*x47) - x54;
  g_out[2] = x54;
  g_out[3] = x39;
  g_out[4] = x44;
  g_out[5] = x37;
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

