/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_config.hpp
 * @brief     evaluation configuration
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 *            2024-12-09 update parking evaluation to ROS2 from 2023jasiple (sunghoon8585@gmail.com)
 */

#ifndef __EVALUATION_CONFIG_HPP__
#define __EVALUATION_CONFIG_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef enum {
    APPROACHING = 0,
    IN_PROGRESS,
    SUCCESS,
    FAIL
} ParkingStatus;

typedef struct {
    unsigned char status = ParkingStatus::APPROACHING;
    double parking_success_threshold;
    
    double slot_0_x;
    double slot_0_y;
    double slot_1_x;
    double slot_1_y;
    double slot_2_x;
    double slot_2_y;

    double region_x;        // not used
    double region_y;        // not used
} ParkingSlotParams;


typedef struct {
    std::string ref_csv_path;
    std::string lane_id;

    double loop_rate_hz;
    double eval_time_limit;
    double eval_lane_departure;

    ParkingSlotParams parking;
} EvaluationConfig;


#endif // __EVALUATION_CONFIG_HPP__