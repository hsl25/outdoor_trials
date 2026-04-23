#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <stdio.h>
#include <cstdint>
#include <vector>

#include "servo.hpp"
#include "tof.hpp"
#include "IMU.hpp"

#define DEG_TO_RAD 0.01745329251 // One radian
#define RAD_TO_DEG 57.2957795131
#define PEAK_CHECKS 3 // Number of times we check whether the data is increasing consecutively 
#define ROVER_WIDTH 600 // Width of the rover in mm
#define ROVER_LENGTH 440 // Length of rover in mm
#define SAFETY_MARGIN 10 // 10mm safety margin added to the width and length of the rover
#define ALPHA 0.2 // Safety constant
#define SKID_CHECK_ANGLE 90
#define DISTANCE_DIVIDER 2

class Navigation {
    public:
        Navigation();
        void initial_sweep(int num_sweeps, uint16_t lidar_buf[], int size);
        float calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2);
        std::vector<int> calc_peaks(uint16_t arr[], int size);
        int calc_min_sweep_angle(float dist);
        std::vector<float> calc_gap_width(std::vector<int> peak_angles, std::vector<int> min_sweep_angles, uint16_t buf[], int size);
        int choose_direction(std::vector<float> gaps);
        void skid_into_position(float start_yaw, float final_yaw);
        void reset_buffer(uint16_t lidar_buff[], int size);
    private:
};

#endif