#include<baichuan_arm_dynamics_models/elfin/elfin3_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin3Pose1DynamicsModel::gravityTerm( double* g_out, const double* parms, const double* q )
{
  double x0 = cos(q[1]);
  double x1 = cos(q[3]);
  double x2 = sin(q[1]);
  double x3 = 9.788*x2;
  double x4 = cos(q[2]);
  double x5 = 9.788*x0;
  double x6 = sin(q[2]);
  double x7 = -x6;
  double x8 = x3*x4 + x5*x7;
  double x9 = x1*x8;
  double x10 = sin(q[4]);
  double x11 = cos(q[4]);
  double x12 = x3*x6 + x4*x5;
  double x13 = -x12;
  double x14 = -x13;
  double x15 = x10*x9 + x11*x14;
  double x16 = cos(q[5]);
  double x17 = sin(q[3]);
  double x18 = -x17;
  double x19 = x18*x8;
  double x20 = x10*x13 + x11*x9;
  double x21 = sin(q[5]);
  double x22 = -x21;
  double x23 = x16*x19 + x20*x22;
  double x24 = parms[72]*x15 - parms[73]*x23;
  double x25 = -x19;
  double x26 = x16*x20 + x19*x21;
  double x27 = -x15;
  double x28 = parms[71]*x27 + parms[73]*x26;
  double x29 = parms[74]*x26;
  double x30 = -0.1975*x21;
  double x31 = parms[74]*x23;
  double x32 = x16*x31;
  double x33 = parms[59]*x15 + parms[60]*x25 + x16*x24 + x22*x28 + x29*x30 - 0.1975*x32;
  double x34 = parms[71]*x23 - parms[72]*x26;
  double x35 = parms[58]*x19 - parms[59]*x20 + x34;
  double x36 = -x11;
  double x37 = -parms[45]*x25 - parms[47]*x9 - x10*x33 - x35*x36;
  double x38 = -parms[33]*x8 + x37;
  double x39 = parms[46]*x19 + parms[47]*x14 + x10*x35 + x11*x33;
  double x40 = x16*x29;
  double x41 = parms[58]*x27 + parms[60]*x20 + x16*x28 + x21*x24 + x30*x31 + 0.1975*x40;
  double x42 = parms[45]*x13 - parms[46]*x9 + x41;
  double x43 = parms[48]*x19 + parms[61]*x19 + x21*x29 + x32;
  double x44 = -0.324*x43;
  double x45 = parms[61]*x20 + x22*x31 + x40;
  double x46 = parms[61]*x15 + parms[74]*x15;
  double x47 = parms[48]*x9 + x10*x46 + x11*x45;
  double x48 = x17*x47;
  double x49 = parms[33]*x12 + x1*x39 + x1*x44 + x18*x42 - 0.324*x48;
  double x50 = x1*x47;
  double x51 = parms[32]*x13 + parms[34]*x8 + x1*x42 + x17*x39 + x17*x44 + 0.324*x50;
//
  g_out[0] = x0*(-parms[21]*x3 + x38*x4 + x49*x7) + x2*(-0.266*((x0)*(x0)) - 0.266*((x2)*(x2)))*(x1*x43 + x48) + x2*(parms[21]*x5 + x38*x6 + x4*x49);
  g_out[1] = -parms[19]*x3 + parms[20]*x5 - 0.266*parms[22]*x3 - 0.266*x4*(parms[35]*x8 + x18*x43 + x50) - x51 - 0.266*x6*(parms[35]*x12 - parms[48]*x13 - x10*x45 - x36*x46);
  g_out[2] = x51;
  g_out[3] = x37;
  g_out[4] = x41;
  g_out[5] = x34;
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

