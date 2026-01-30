#ifndef BAICHUAN_ARM_BASIC_DYN_MODEL_H
#define BAICHUAN_ARM_BASIC_DYN_MODEL_H


#include <vector>
#include <math.h>
#include <memory>
#include <Eigen/Dense>

namespace baichuan {

namespace arm {

class BasicDynamicsModel{

public:
  BasicDynamicsModel()
  {

  }

  virtual ~BasicDynamicsModel()
  {

  }

  virtual void regressorYb( double* yb_out, const double* q, const double* dq, const double* ddq ){}
  virtual void regressorY( double* y_out, const double* q, const double* dq, const double* ddq ){}
  virtual void tau( double* tau_out, const double* parms, const double* q, const double* dq, const double* ddq ){}
  virtual void gravityTerm( double* g_out, const double* parms, const double* q ){}
  virtual void frictionTerm( double* f_out, const double* parms, const double* q, const double* dq ){}
  virtual void coriolisTerm( double* c_out, const double* parms, const double* q, const double* dq ){}
  virtual void coriolisMatrix( double* c_out, const double* parms, const double* q, const double* dq ){}
  virtual void inertiaMatrixB( double* b_out, const double* parms, const double* q ){}

  virtual int get_robot_dof(){}

  virtual int get_params_total_amount(){}
  virtual int get_basic_params_amount(){}
  virtual std::vector<size_t> get_basic_params_index(){}

  virtual int get_gravity_basic_params_amount(){}
  virtual std::vector<size_t> get_gravity_basic_params_index(){}
  virtual std::vector<size_t> get_coulomb_friction_params_index(){}
  virtual std::vector<size_t> get_viscous_friction_params_index(){}
  virtual std::vector<double> get_gravity_vector(){}

  std::vector<double> dynamics_params_;
  bool init_state_ = false;

  void check_dof_size(std::vector<double> data)
  {
    if (data.size() != get_robot_dof())
    {
      fprintf(stderr, "Input data size is not equal to %d\n",  get_robot_dof());
      throw "Input data size is not equal to %d",  get_robot_dof();
    }
    
  }

  bool init(std::vector<double> basic_params)
  {
    if (basic_params.size() != get_basic_params_amount())
    {
      fprintf(stderr, "Input basic_param size is %zu, which is suppose to %d\n", basic_params.size(), get_basic_params_amount());
      init_state_ = false;
      return false;
    }
    
    dynamics_params_.resize(get_params_total_amount(), 0.0);
    for (int i = 0; i < get_basic_params_amount(); i++)
    {
      dynamics_params_[get_basic_params_index()[i]] = basic_params[i];
    }
    init_state_ = true;
    return true;
  }

  Eigen::MatrixXd regressorYb(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    check_dof_size(ddq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    
    Eigen::MatrixXd regressor_yb;
    regressor_yb.resize(get_robot_dof(), get_basic_params_amount());

    double* regressor_yb_elements = new double[get_robot_dof()*get_basic_params_amount()];
    regressorYb(regressor_yb_elements, &q[0], &dq[0], &ddq[0]);

    regressor_yb = Eigen::Map<Eigen::MatrixXd>(&regressor_yb_elements[0], get_basic_params_amount(), get_robot_dof()).transpose();
    delete [] regressor_yb_elements;
    return regressor_yb;
  }

  Eigen::MatrixXd regressorY(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    check_dof_size(ddq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    Eigen::MatrixXd regressor_y;
    regressor_y.resize(get_robot_dof(), get_params_total_amount());

    double* regressor_y_elements = new double[get_robot_dof()*get_params_total_amount()];
    regressorYb(regressor_y_elements, &q[0], &dq[0], &ddq[0]);

    regressor_y = Eigen::Map<Eigen::MatrixXd>(&regressor_y_elements[0], get_params_total_amount(), get_robot_dof()).transpose();
    delete [] regressor_y_elements;
    return regressor_y;
  }

  Eigen::VectorXd tau(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    check_dof_size(ddq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    double* elements = new double[get_robot_dof()];
    tau(elements, &dynamics_params_[0], &q[0], &dq[0], &ddq[0]);
    Eigen::VectorXd vector = Eigen::Map<Eigen::VectorXd>(&elements[0], get_robot_dof(), 1);
    delete [] elements;
    return vector;
  }

  Eigen::VectorXd gravityTerm(std::vector<double> q)
  {
    check_dof_size(q);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    double* elements = new double[get_robot_dof()];
    gravityTerm(elements, &dynamics_params_[0], &q[0]);
    Eigen::VectorXd vector = Eigen::Map<Eigen::VectorXd>(&elements[0], get_robot_dof(), 1);
    delete [] elements;
    return vector;
  }
  
  Eigen::VectorXd frictionTerm(std::vector<double> q, std::vector<double> dq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    double* elements = new double[get_robot_dof()];
    frictionTerm(elements, &dynamics_params_[0], &q[0], &dq[0]);
    Eigen::VectorXd vector = Eigen::Map<Eigen::VectorXd>(&elements[0], get_robot_dof(), 1);
    delete [] elements;
    return vector;
  }
  
  Eigen::VectorXd FrictionCompensationwithstart(std::vector<double> q, 
                                                std::vector<double> dq,
                                                std::vector<double> v_ref,
                                                double k_coulomb) //静摩擦通常大于动摩擦，设定来补偿启动力矩
                                                
  {
    check_dof_size(q);
    check_dof_size(dq);
    check_dof_size(v_ref);

    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }

    Eigen::VectorXd vector = Eigen::VectorXd::Zero(get_robot_dof());

    for (size_t i = 0; i < get_robot_dof(); ++i)
    {
      if(std::abs(dq[i]) < 1e-5 && std::abs(v_ref[i]) > 1e-5)  //状态为0,但是期望不为零，判断为启动状态
      {
        vector[i] = dynamics_params_[get_viscous_friction_params_index()[i]] * v_ref[i]
                  + dynamics_params_[get_coulomb_friction_params_index()[i]] * (((v_ref[i]) > 0) - ((v_ref[i]) < 0)); //使用期望作为补偿
        vector[i] *= k_coulomb;//放大以克服静摩擦力
      }
      else 
      {
        vector[i] = dynamics_params_[get_viscous_friction_params_index()[i]] * dq[i]
                  + dynamics_params_[get_coulomb_friction_params_index()[i]] * (((dq[i]) > 0) - ((dq[i]) < 0));
      }
    }
    return vector;
  }

  Eigen::VectorXd coriolisTerm(std::vector<double> q, std::vector<double> dq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    double* elements = new double[get_robot_dof()];
    coriolisTerm(elements, &dynamics_params_[0], &q[0], &dq[0]);
    Eigen::VectorXd vector = Eigen::Map<Eigen::VectorXd>(&elements[0], get_robot_dof(), 1);
    delete [] elements;
    return vector;
  }

  Eigen::MatrixXd coriolisMatrix(std::vector<double> q, std::vector<double> dq)
  {
    check_dof_size(q);
    check_dof_size(dq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    Eigen::MatrixXd coriolis_matrix;
    coriolis_matrix.resize(get_robot_dof(), get_robot_dof());

    double* coriolis_matrix_elements = new double[get_robot_dof()*get_robot_dof()];
    coriolisMatrix(coriolis_matrix_elements, &dynamics_params_[0], &q[0], &dq[0]);

    coriolis_matrix = Eigen::Map<Eigen::MatrixXd>(&coriolis_matrix_elements[0], get_robot_dof(), get_robot_dof()).transpose();
    delete [] coriolis_matrix_elements;
    return coriolis_matrix;
  }

  Eigen::MatrixXd inertiaMatrixB(std::vector<double> q)
  {
    check_dof_size(q);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    Eigen::MatrixXd inertia_matrix;
    inertia_matrix.resize(get_robot_dof(), get_robot_dof());

    double* inertia_matrix_elements = new double[get_robot_dof()*get_robot_dof()];
    inertiaMatrixB(inertia_matrix_elements, &dynamics_params_[0], &q[0]);

    inertia_matrix = Eigen::Map<Eigen::MatrixXd>(&inertia_matrix_elements[0], get_robot_dof(), get_robot_dof()).transpose();
    delete [] inertia_matrix_elements;
    return inertia_matrix;
  }

  Eigen::VectorXd coulombFriction(std::vector<double> dq)
  {
    check_dof_size(dq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(get_robot_dof());
    for (size_t i = 0; i < get_robot_dof(); i++)
    {
      vector[i] = dynamics_params_[get_coulomb_friction_params_index()[i]] * (((dq[i]) > 0) - ((dq[i]) < 0));
    }
    return vector;
  }

  Eigen::VectorXd viscousFriction(std::vector<double> dq)
  {
    check_dof_size(dq);
    if (!init_state_)
    {
      fprintf(stderr, "Initialize model first\n");
      throw "Initialize model first";
    }
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(get_robot_dof());
    for (size_t i = 0; i < get_robot_dof(); i++)
    {
      vector[i] = dynamics_params_[get_viscous_friction_params_index()[i]] * dq[i];
    }
    return vector;
  }
};

} // end namespace robot_dynamics
} // end namespace moying

#endif // end BAICHUAN_ARM_ELFIN_BASIC_DYN_MODEL_H
