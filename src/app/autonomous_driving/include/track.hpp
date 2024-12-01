#ifndef TRACK_HPP
#define TRACK_HPP

#include <string>
#include <iostream>
#include <stdlib.h>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// Eigen
#include <eigen3/Eigen/Dense>

#include "kalman_filter.hpp"  // KalmanFilter 헤더파일
#include "readparam.hpp"      // Param 클래스가 정의된 파일의 헤더 파일

struct Detect {
    int classname = 0;
    float yaw;
    float velocity;
    Eigen::VectorXd position;  // x, y
};

struct TrackState {
    int Confirmed = 1;
    int UnConfirmed = 2;
    int Delete = 3;
};

class Track {
public:
    // 생성자에서 KalmanFilter 객체 초기화
    Track(int id, float time, Detect& det)
        : id_(id) {

        // KalmanFilter 객체를 생성하고, 초기 상태로 설정
        ekf_ = std::make_shared<KalmanFilter>();

        // Detect 객체에서 받은 값으로 초기 상태 설정
        float x = det.position(0);
        float y = det.position(1);
        float yaw = det.yaw;
        float velocity = det.velocity;
        
        // 초기 상태를 KalmanFilter로 전달
        Eigen::Vector4d initial_state(x, y, yaw, velocity);
        ekf_->init(initial_state);

        track_state_ = TrackState::UnConfirmed;
        age_ = 0;
        measure_ << x, y, yaw, velocity;
    }

    ~Track() {}

    int GetId() const {
        return id_;
    }

    // 예측 (Prediction)
    void Prediction(float dt, float velocity, float yaw_rate) {
        age_++;
        // KalmanFilter를 사용하여 예측 수행
        ekf_->predict(dt, velocity, yaw_rate);  // dt, 속도, 회전율 전달
    }

    // 업데이트 (Update) - 여러 개의 탐지 객체가 있을 때
    void Update(std::vector<Eigen::VectorXd>& det) {
        // 측정값을 KalmanFilter로 업데이트
        for (auto& d : det) {
            Eigen::Vector4d measurement(d(0), d(1), d(2), d(3));  // x, y, yaw, velocity
            ekf_->update(measurement);  // KalmanFilter 업데이트
        }

        age_ = 0;
        hit_ += 1;
        if (track_state_ == TrackState::UnConfirmed || hit_ > max_init_) {
            track_state_ = TrackState::Confirmed;
        }
    }

    // 업데이트 (Update) - 하나의 탐지 객체가 있을 때
    void Update(Eigen::VectorXd& det) {
        // KalmanFilter를 사용하여 업데이트
        Eigen::Vector4d measurement(det(0), det(1), det(2), det(3));  // x, y, yaw, velocity
        ekf_->update(measurement);

        age_ = 0;
        hit_ += 1;
        if (track_state_ == TrackState::UnConfirmed || hit_ > max_init_) {
            track_state_ = TrackState::Confirmed;
        }
    }

    int Age() const {
        return age_;
    }

    void MarkMissed() {
        if (track_state_ == TrackState::UnConfirmed) {
            track_state_ = TrackState::Delete;
        } else if (age_ > max_age_) {
            track_state_ = TrackState::Delete;
        }
    }

    int GetTrackState() const {
        return track_state_;
    }

    // 상태 벡터 반환
    Eigen::Vector4d GetState() {
        return ekf_->getState();  // KalmanFilter에서 상태 벡터 반환
    }

    Eigen::Matrix4d GetP() {
        return ekf_->getP();  // Kalman Filter의 오차 공분산 행렬
    }

    Eigen::Vector4f GetMeasure() const {
        return measure_;
    }

    void UpdateMeasure(float x, float y) {
        measure_(0) = x;
        measure_(1) = y;
        measure_(2) = yaw;
        measure_(3) = velocity;
    }

private:
    std::shared_ptr<KalmanFilter> ekf_;  // KalmanFilter 객체
    Param param_;
    int id_ = -1;
    int age_ = 0;  // Time since last update
    int max_age_ = 4;
    int hit_ = 1;  // Tracking hit count
    int max_init_ = 3;
    int track_state_ = 0;
    Eigen::Vector4f measure_;
    TrackState TrackState_;  // Track state (Confirmed, UnConfirmed, Delete)
};

#endif  // TRACK_HPP
