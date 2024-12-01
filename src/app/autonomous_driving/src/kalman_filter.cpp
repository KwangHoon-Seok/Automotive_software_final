#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    // F = Eigen::Matrix3d::Identity();
    F = Eigen::Matrix4d::Identity();
    H = Eigen::Matrix4d::Identity(); // 관측 행렬을 단위행렬로 설정
    Q = Eigen::Matrix4d::Identity()*0.0001;
    // Q(2,2) = 0.0001;
    // G = Eigen::Matrix<double, 3, 2>::Zero(); // G를 초기화    
    // u = Eigen::Vector2d::Zero();
    R = Eigen::Matrix4d::Identity() * 1;    // R은 고정
    R(2,2) = 1 * M_PI / 180;
    P = Eigen::Matrix4d::Identity()*1000;
    x = Eigen::Vector4d::Zero();
}

void KalmanFilter::init(const Eigen::Vector4d& initial_state) {
    x = initial_state;
}

void KalmanFilter::predict(double dt, double v, double yaw_rate) {
    Q = Eigen::Matrix4d::Identity()*0.0001;
    // G(0,0) = v * dt * cos(x(2));     
    // G(1,0) = v * dt * sin(x(2));
    // G(2,1) = dt;
    
    // u(0) = v;
    // u(1) = yaw_rate;
    
    F(0,2) = -x(3)*dt*sin(x(2));
    F(1,2) = x(3)*dt*cos(x(2));

    // 상태 예측
    x(0) += x(3) *dt * cos(x(2));
    x(1) += x(3) *dt * sin(x(2));
    x(2) += yaw_rate *dt;
    x(3) = v;
    // if (abs(yaw_rate) > 0.08){
    //     Q(0,0) += abs(yaw_rate * dt);
    //     Q(1,1) += abs(yaw_rate * dt);
    //     Q(2,2) += abs(yaw_rate * dt);
    // }

    // 오차 공분산 행렬 업데이트
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Vector4d& measurement) {
    // 예측 오차 계산
    
    Eigen::Vector4d y = measurement - H * x;
    while(y(2)> M_PI){
        y(2) -= 2 * M_PI;
    }
    while(y(2) < - M_PI){
        y(2) += 2 * M_PI;
    }
    

    Eigen::Matrix4d S = H * P * H.transpose() + R;
    Eigen::Matrix4d K = P * H.transpose() * S.inverse();

    // 상태 벡터 업데이트
    x = x + K * y;

    // 오차 공분산 행렬 업데이트
    P = (Eigen::Matrix4d::Identity() - K * H) * P;
}

Eigen::Vector4d KalmanFilter::getState() const {
    return x;
}

Eigen::Matrix4d KalmanFilter::getP() const{
    return P;
}
