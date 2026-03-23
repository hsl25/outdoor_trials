#include <iostream>
#include <stdio.h>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <vector>

#include "buffer.hpp"

Buffer::Buffer() {
    // Initialise variables 
    end_of_sweep = 0;
    head = 0;
    calib_index = 0;
    buffer_count = 0;
    rolling_mean = 0.0;
    rolling_std_dev = 0.0;
    variance = 0.0;

    for (size_t i = 0; i < N; i++) {
        lidar_buffer[i] = 0;
    }
        
}

void Buffer::add_rolling_sample(uint16_t sample) {
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

void Buffer::add_calib_sample(uint16_t sample) {
    calib_buffer[calib_index] = sample;

    // Update sample number accordingly
    // The servo will sweep from 0 degrees to 180 degrees, then 180 degrees back to 0 degrees
    // Therefore, the index in the buffer has to go from 0 to 179, then 179 back down to 0
    if (end_of_sweep == 0) {
        if (calib_index == 179) {
            end_of_sweep = !end_of_sweep;
        } else {
            calib_index++;
        }
    }

    if (end_of_sweep == 1) {
        if (calib_index == 0) {
            end_of_sweep = !end_of_sweep;
        } else {
            calib_index--;
        }
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
