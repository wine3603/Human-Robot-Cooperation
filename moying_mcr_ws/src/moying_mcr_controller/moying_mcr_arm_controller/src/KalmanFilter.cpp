#include <iostream>
#include <vector>
#include <Eigen/Dense> // 使用 Eigen 库处理矩阵运算
#include <velocity_interface_controller/KalmanFilter.h> // 使用 Eigen 库处理矩阵运算
namespace velocity_interface_controller{

using namespace Eigen;
using namespace std;

KalmanFilter::KalmanFilter(int state_dim, int measure_dim, int control_dim)
        : state_dim_(state_dim), measure_dim_(measure_dim), control_dim_(control_dim)
{
    A = MatrixXd::Identity(state_dim, state_dim); // 状态转移矩阵
    B = MatrixXd::Zero(state_dim, control_dim);  // 控制输入矩阵
    H = MatrixXd::Zero(measure_dim, state_dim);  // 测量矩阵

    Q = MatrixXd::Identity(state_dim, state_dim) * 0.01; // 过程噪声协方差矩阵
    R = MatrixXd::Identity(measure_dim, measure_dim) * 1.0; // 测量噪声协方差矩阵
    P = MatrixXd::Identity(state_dim, state_dim); // 估计误差协方差矩阵

    X = VectorXd::Zero(state_dim); // 状态估计向量

    // 初始化矩阵 B 和 H
    for (int i = 0; i < control_dim; ++i) {
        B(i + 6, i) = 1.0; // 控制输入直接影响速度
    }

    // 测量矩阵 H 直接从速度测量中提取速度
    for (int i = 0; i < measure_dim; ++i) {
        H(i, i + 6) = 1; // 速度的测量对应状态向量的后半部分
    }
}

void KalmanFilter::setMatrices(const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, 
                     const MatrixXd& Q, const MatrixXd& R, const MatrixXd& P)
{
    this->A = A;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->P = P;
}

VectorXd KalmanFilter::update(const VectorXd& measurement, const VectorXd& control)
{
    X = A * X + B * control;
    P = A * P * A.transpose() + Q;

    // Update step
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse(); // Kalman Gain

    X = X + K * (measurement - H * X);
    P = (MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P;

    return X;
}

}