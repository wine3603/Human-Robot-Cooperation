#ifndef BAICHUAN_ARM_ELFIN10L_DYN_MODEL_H
#define BAICHUAN_ARM_ELFIN10L_DYN_MODEL_H

#include <moying_mcr_arm_dynamics_models/basic_dynamics_model.h>

namespace baichuan {

namespace arm {

namespace elfin{

class Elfin10lPose1DynamicsModel : public baichuan::arm::BasicDynamicsModel
{

public:
  Elfin10lPose1DynamicsModel();
  ~Elfin10lPose1DynamicsModel();

  void regressorYb( double* yb_out, const double* q, const double* dq, const double* ddq );
  void regressorY( double* y_out, const double* q, const double* dq, const double* ddq );
  void tau( double* tau_out, const double* parms, const double* q, const double* dq, const double* ddq );
  void gravityTerm( double* g_out, const double* parms, const double* q );
  void frictionTerm( double* f_out, const double* parms, const double* q, const double* dq );
  void coriolisTerm( double* c_out, const double* parms, const double* q, const double* dq );
  void coriolisMatrix( double* c_out, const double* parms, const double* q, const double* dq );
  void inertiaMatrixB( double* b_out, const double* parms, const double* q );

  int get_robot_dof();

  int get_params_total_amount();
  int get_basic_params_amount();
  std::vector<size_t> get_basic_params_index();

  int get_gravity_basic_params_amount();
  std::vector<size_t> get_gravity_basic_params_index();
  std::vector<size_t> get_coulomb_friction_params_index();
  std::vector<size_t> get_viscous_friction_params_index();
  std::vector<double> get_gravity_vector();

private:
  int robot_dof_;

  int params_total_amount_;
  int basic_params_amount_;
  std::vector<size_t> basic_params_index_;

  int gravity_basic_params_amount_;
  std::vector<size_t> gravity_basic_params_index_;
  std::vector<double> gravity_vector_;

  std::vector<size_t> coulomb_friction_params_index_;
  std::vector<size_t> viscous_friction_params_index_;
};

} // end namespace robot_dynamics
}
} // end namespace moying

#endif // end BAICHUAN_ARM_ELFIN3_DYN_MODEL_H
