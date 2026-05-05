#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <stdio.h>
#include <cstdint>
#include <vector>

#include "servo.hpp"
#include "tof.hpp"
#include "IMU.hpp"
#include "driving.hpp"
#include "matrix.hpp"
#include "buffer.hpp"

#define DEG_TO_RAD 0.01745329251 // One radian
#define RAD_TO_DEG 57.2957795131
#define PEAK_CHECKS 3 // Number of times we check whether the data is increasing consecutively 
#define ROVER_WIDTH 600 // Width of the rover in mm
#define ROVER_LENGTH 440 // Length of rover in mm
#define SAFETY_MARGIN 100 // 10mm safety margin added to the width and length of the rover
#define ALPHA 0.2 // Safety constant
#define DISTANCE_DIVIDER 2
#define ANGLE_STEP 3
#define MIN_GAP_DISTANCE 800
#define MIN_GAP_CONFIRM 5 // Consecutive number of points that must all exceed threshold distance before we declare a gap has opened
#define GAP_HYSTERESIS 3 // Consecutive number of points below threshold before a gap closes
#define MAX_STEP_CHANGE 150 // mm - max allowed drop between adjacent points before we treat it as an obstacle edge
#define MAX_STEP_FRACTION 0.20f   // 20% drop from previous point = obstacle edge
#define VARIANCE_WINDOW 5
#define MAX_GAP_VARIANCE 8000
#define NUM_CHECKS 5
#define SKID_DEADBAND_DEG 2.0f 
#define SKID_STOP_FRACTION  0.90f 
#define THRESHOLD_DIST 500
#define THRESHOLD_RATIO 0.8

struct Gap {
    int start_angle;    // buffer index where gap opens
    int end_angle;      // buffer index where gap closes
    float start_dist;     // distance at opening edge
    float end_dist;       // distance at closing edge
    float width;          // chord width in mm
};

struct BestGap {
    bool  valid;
    int   centre_index;   // buffer index of gap centre
    float width_mm;       // chord width of the gap
    float centre_dist_mm; // distance reading at the centre
    float imu_angle_deg;  // heading delta to aim for (negative=left, positive=right)
};

class Navigation {
    public:
        Navigation(TOF& tof, Servo& sv, IMU& imu, Drive& dr);
        void median_filter(uint16_t buf[], int size);
        std::vector<Gap> find_gaps(uint16_t buf[], int size);
        BestGap find_best_gap(uint16_t buf[], int size);
        void forward_sweep(int num_sweeps, uint16_t lidar_buf[], int size);
        void rear_sweep(int num_sweeps, uint16_t lidar_buf[], int size);
        void print_buffer(uint16_t buf[], int size);
        float calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2);
        std::vector<int> calc_peaks(uint16_t arr[], int size);
        int calc_min_sweep_angle(float dist);
        std::vector<float> calc_gap_width(std::vector<int> peak_angles, std::vector<int> min_sweep_angles, uint16_t buf[], int size);
        int choose_direction(std::vector<float> gaps);
        void skid_into_position(float delta_deg);
        void reset_buffer(uint16_t lidar_buff[], int size);
        bool space_check(uint16_t buf[], int size);
        bool check_openings(uint16_t buf[], int size, int min_dist);
        int check_max_range(uint16_t buf[], int size, int dist_choice);
        std::vector<int> detect_edge(uint16_t buf[], int size);
        int max_index(uint16_t buf[], int size);
    private:
        TOF& tof_;
        Servo& sv_;
        IMU& imu_;
        Drive& dr_;
};

#endif