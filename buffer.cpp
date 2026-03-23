#include <iostream>
#include <stdio.h>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <vector>

#include "buffer.hpp"

Buffer::Buffer() {
    // Initialise variables 
    head = 0;
    buffer_count = 0;
    rolling_mean = 0.0;
    rolling_std_dev = 0.0;
    variance = 0.0;

    for (size_t i = 0; i < N; i++) {
        lidar_buffer[i] = 0;
    }
        
}

void Buffer::add_sample(uint16_t sample) {
    // Samples are added into a circular buffer 
    // The data point at the head is replaced with the new data point
    // The head shift accordingly 
    lidar_buffer[head] = sample;
    ++head;
    head = head % N;

    if (buffer_count < N) {
        buffer_count++;
    }

}

void Buffer::compute_stats() {
    if (buffer_count < N) {
        return;
    }

    rolling_mean = 0.0;
    variance = 0.0;

    for (size_t i = 0; i < N; i++)
        rolling_mean += lidar_buffer[i];

    rolling_mean /= N;

    for (size_t i = 0; i < N; i++) {
        double diff = lidar_buffer[i] - rolling_mean;
        variance += diff * diff;
    }

    variance /= (N - 1);
    rolling_std_dev = std::sqrt(variance);
}

// Check the data in the buffer using the basis limits of mu +/- k*sigma
bool Buffer::is_anomaly(uint16_t data) {
    if (buffer_count < N)
        return false;

    double lower = rolling_mean - (K * rolling_std_dev);
    double upper = rolling_mean + (K * rolling_std_dev);

    // Returns true if it is an anomaly (i.e., out of range) and false if it is in range 
    return (data < lower || data > upper);
}

// Check the data in the buffer for large jumps 
bool Buffer::is_big_jump(uint16_t new_sample) {
    if (buffer_count < 1) {
        return false;
    }

    size_t last = (head + N - 1) % N;

    double diff = std::abs((double)new_sample - (double)lidar_buffer[last]);

    return diff > (K * rolling_std_dev);
}

bool Buffer::is_ready() {
    return buffer_count == N;
}

// This function creates a fixed (not rolling) buffer for calibration for before the rover starts moving
std::vector<int> Buffer::create_calib_buffer(int num_data) {
    std::vector<int> arr(num_data,0);
    return arr;
}