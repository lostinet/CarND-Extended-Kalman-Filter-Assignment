#include "kalman_filter.h"
#include <math.h>
//#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    
    // state vector
    x_ = F_ * x_;
    
    // Covariance Matrix P
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    
    VectorXd y = z - H_ * x_;
    
    UpdateRest(y,H_,R_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    VectorXd h_x = CartesianToPolar(x_);
    
    VectorXd y = z - h_x;
    
//    Tools tools;
//    Hj_radar_ = tools.CalculateJacobian(x_);
    
    UpdateRest(y,H_,R_);
}


void KalmanFilter::UpdateRest(const VectorXd &y_in,const MatrixXd &H_in,const MatrixXd &R_in)
{
    MatrixXd Ht = H_in.transpose();
    
    MatrixXd S = H_in * P_ * Ht + R_in;
    
    MatrixXd Si = S.inverse();
    
    MatrixXd K = P_ * Ht * Si;
    
    x_ = x_ + (K * y_in);
    
    VectorXd I = MatrixXd::Identity(4,4);
    
    P_ = (I - K * H_in) * P_;
    
}


VectorXd KalmanFilter::CartesianToPolar(const VectorXd &x_state)
{
    VectorXd h_x = VectorXd(3);
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float px_py = pow(px,2) + pow(py,2);
    
    if(fabs(px_py)<0.0001)
    {
        px_py = 0.0001;
    }
    float sqrt_px_py = sqrt(px_py);
    
    h_x << sqrt_px_py,atan2(py,px),(px*vx + py*vy)/sqrt_px_py;
    
    return h_x;
}




