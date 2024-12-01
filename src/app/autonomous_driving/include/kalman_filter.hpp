#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter {
public:
    KalmanFilter();
    void init(const Eigen::Vector4d& initial_state);
    void predict(double dt, double v, double yaw);
    void update(const Eigen::Vector4d& measurement); // Vector3d 사용
    Eigen::Vector4d getState() const;
    Eigen::Matrix4d getP() const;

private:
    Eigen::Matrix4d F;
    // Eigen::Matrix4d A; // 상태 전이 행렬
    Eigen::Matrix4d P; // 오차 공분산 행렬
    Eigen::Matrix4d H; // 관측 행렬
    Eigen::Matrix4d Q; // 프로세스 잡음 공분산 행렬s
    Eigen::Matrix4d R; // 측정 잡음 공분산 행렬
    Eigen::Vector4d x; // 상태 벡터
    // Eigen::Matrix<double, 3, 2> G;
    // Eigen::Vector4d u;
};


#endif // KALMANFILTER_H
