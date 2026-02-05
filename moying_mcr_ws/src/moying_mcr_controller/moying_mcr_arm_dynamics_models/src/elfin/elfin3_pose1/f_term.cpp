#include<moying_mcr_arm_dynamics_models/elfin/elfin3_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin3Pose1DynamicsModel::frictionTerm( double* f_out, const double* parms, const double* q, const double* dq )
{
//
  f_out[0] = dq[0]*parms[11] + parms[12]*(((dq[0]) > 0) - ((dq[0]) < 0));
  f_out[1] = dq[1]*parms[24] + parms[25]*(((dq[1]) > 0) - ((dq[1]) < 0));
  f_out[2] = dq[2]*parms[37] + parms[38]*(((dq[2]) > 0) - ((dq[2]) < 0));
  f_out[3] = dq[3]*parms[50] + parms[51]*(((dq[3]) > 0) - ((dq[3]) < 0));
  f_out[4] = dq[4]*parms[63] + parms[64]*(((dq[4]) > 0) - ((dq[4]) < 0));
  f_out[5] = dq[5]*parms[76] + parms[77]*(((dq[5]) > 0) - ((dq[5]) < 0));
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

