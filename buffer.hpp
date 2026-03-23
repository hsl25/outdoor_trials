#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <vector>

// Other defines 
#define N 10 // Number of samples in a window for filtering
#define K 1 // Tuneable constant for filtering - this is the scalar for the standard deviation
#define MAX_SERVO_ANGLE 180 // This is the maximum angle scope of the servo

class Buffer {
    public:
        Buffer();
        void add_rolling_sample(uint16_t sample);
        void add_calib_sample(uint16_t sample);
        void compute_stats();
        bool is_big_jump(uint16_t new_sample);
        bool is_anomaly(uint16_t data);
        bool is_ready();
        
        private:
        // Circular buffer 
        uint16_t lidar_buffer[N];
        uint16_t filtered_data[N];
        uint16_t calib_buffer[MAX_SERVO_ANGLE];
        size_t calib_index;
        size_t head;
        bool end_of_sweep;

        // Statistics
        double rolling_mean;
        double rolling_std_dev;
        double variance;
        int buffer_count;

};

#endif