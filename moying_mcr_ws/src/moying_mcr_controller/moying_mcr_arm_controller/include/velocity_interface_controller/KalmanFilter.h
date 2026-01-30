#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense> // 使用 Eigen 库处理矩阵运算
namespace velocity_interface_controller{

using namespace Eigen;
using namespace std;
class KalmanFilter {
public:
    KalmanFilter(int state_dim, int measure_dim, int control_dim);
    void setMatrices(const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, 
                     const MatrixXd& Q, const MatrixXd& R, const MatrixXd& P);
    VectorXd update(const VectorXd& measurement, const VectorXd& control);

private:
    int state_dim_;
    int measure_dim_;
    int control_dim_;
    MatrixXd A, B, H, Q, R, P;
    VectorXd X;
};

}
