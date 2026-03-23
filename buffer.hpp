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


class Buffer {
    public:
        Buffer();
        void add_sample(uint16_t sample);
        void compute_stats();
        bool is_big_jump(uint16_t new_sample);
        bool is_anomaly(uint16_t data);
        bool is_ready();
        std::vector<int> create_calib_buffer(int num_data);
        
        private:
        // Circular buffer 
        uint16_t lidar_buffer[N];
        uint16_t filtered_data[N];
        size_t head;

        // Statistics
        double rolling_mean;
        double rolling_std_dev;
        double variance;
        int buffer_count;

};

#endif