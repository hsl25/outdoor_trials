#include <stdio.h>
#include <math.h>
#include <vector>

#include "navigation.hpp"
#include "servo.hpp"
#include "IMU.hpp"
#include "driving.hpp"

// Constructor
Navigation::Navigation(TOF& tof, Servo& sv, IMU& imu, Drive& dr) : tof_(tof), sv_(sv), imu_(imu), dr_(dr) {}

// Start continuous ranging
// This function still needs to be modified for adding data into a buffer and calibrating
// This is how calibration will run:
// 1. The servo will start at position 0 degrees
// 2. The servo will increment 1 degree. It will then indicate via one of something like: toggling a bit, raising a flag, etc. 
// 3. The LiDAR will then check if data is available, and if it is available, it will add it to the correct buffer position
// 4. After data has been successfully added, the servo will increment the angle again and the process will start again

// This function takes as input the lidar buffer and its size, and also the number of times we will sweep back and forth
// It simply performs the sweeping and loads data into the buffer
// The input 'size' to this function requires: size = SERVO_MAX_SWEEP_ANGLE - SERVO_MIN_SWEEP_ANGLE + 1
void Navigation::forward_sweep(int num_sweeps, uint16_t lidar_buf[], int size) {
    // Keep track of the number of sweeps the servo has done
    int num = 0;

    std::vector<uint32_t> accum(size, 0);

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num < num_sweeps) {
        for (int i = SERVO_MIN_SWEEP_ANGLE; i < size + SERVO_MIN_SWEEP_ANGLE; i++) {
            // Now increment the angle of the servo by 1 degree
            sv_.set_front_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(10); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = 0; // tof_.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            accum[i - SERVO_MIN_SWEEP_ANGLE] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        for (int i = size + SERVO_MIN_SWEEP_ANGLE; i > SERVO_MIN_SWEEP_ANGLE; i--) {
            // Now slowly reset the servo back to to the start scan position
            sv_.set_front_angle(i);
            sleep_ms(10);
  
        }
        
        num++;

    }

    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (accum[i] / num_sweeps);
    }

    // For debugging
    // printf("Front sweep complete.\n");

}

// This function takes as input the lidar buffer and its size, and also the number of times we will sweep back and forth
// It simply performs the sweeping and loads data into the buffer
void Navigation::rear_sweep(int num_sweeps, uint16_t lidar_buf[], int size) {
    // Keep track of the number of sweeps the servo has done
    int num = 0;

    std::vector<uint32_t> accum(size, 0);

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num < num_sweeps) {
        for (int i = SERVO_MIN_SWEEP_ANGLE; i < size + SERVO_MIN_SWEEP_ANGLE; i++) {
            // Now increment the angle of the servo by 1 degree
            sv_.set_rear_angle(i);

            // Wait for servo to physically reach the degree
            // sleep_ms(100); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = tof_.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            accum[i - SERVO_MIN_SWEEP_ANGLE] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        // Reset servo angle
        sv_.set_rear_angle(SERVO_MIN_SWEEP_ANGLE);
        sleep_ms(300);
        
        num++;

    }

    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (accum[i] / num_sweeps);
    }

    // For debugging
    // printf("Rear sweep complete.\n");

}

void Navigation::print_buffer(uint16_t buf[], int size) {
    for (int i = 0; i < size; i++) {
        int temp = (int) (buf[i]);
        int angle_to_print = i + SERVO_MIN_SWEEP_ANGLE;
        printf("Angle: %d   Distance(mm): %d \n", angle_to_print, temp);
    }
}

float Navigation::calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2) {
    int real_angle1 = angle1 + SERVO_MIN_SWEEP_ANGLE;
    int real_angle2 = angle2 + SERVO_MIN_SWEEP_ANGLE;
    int theta_diff = angle2 - angle1;
    float x1 = (float) length1;
    float x2 = (float) length2;

    return sqrtf((x1 * x1) + (x2 * x2) - (2 * x1 * x2 * cosf(theta_diff * DEG_TO_RAD)));
}

std::vector<int> Navigation::calc_peaks(uint16_t arr[], int size) {
    // Create a vector to store all the peak angles 
    // We use int because the angles are all integers 
    std::vector<int> peaks;

    for (unsigned int i = 1; i < size - 1; i++) {
        // Keep checking the data until the distances stop increasing
        if((arr[i] > arr[i - 1]) && (arr[i] > arr[i + 1])) {
            peaks.push_back(i);
        }
    }

    return peaks;
    
}

int Navigation::calc_min_sweep_angle(float dist) {
    float temp_angle = 2 * atan((ROVER_WIDTH + SAFETY_MARGIN) / (2 * dist)); // Returns an angle in radians
    temp_angle *= RAD_TO_DEG;

    // Now I need to round up - always round up because if the minimum angle is 30.2 degrees, it is safer to have a larger angle 
    // I can do this by adding 1 and then casting to an int, so the decimal places will be 'chopped off' 
    // I also need to ensure that the minimum angle is even so that I have an even number angle either side of... 
    // ... the 'greatest_distance_angle' 
    int min_sweep_angle = (int) (temp_angle + 1); 

    if (min_sweep_angle % 2 != 0) { 
        min_sweep_angle++; 
    } 

    return min_sweep_angle;
}

// Arguments needed:
// - the vector of peak angles 
// - the vector of minimum sweep angles 
// - the entire LiDAR buffer and its size 
std::vector<float> Navigation::calc_gap_width(std::vector<int> peak_angles, std::vector<int> min_sweep_angles, uint16_t buf[], int size)  {
    // Define a vector for the gap widths
    std::vector<float> gap_widths;
    
    // Define values 
    int num_checks = 3;
    int lt_count = 0; // 'Less-than count'
    int gt_count = 0; // 'Greater-than count'

    float end_distance1 = 0.0f;
    float end_distance2 = 0.0f;
    int end_angle1 = 0;
    int end_angle2 = 0;
    
    // Implement flags to determine if the distances drop below the threshold or not
    bool found_left = false;
    bool found_right = false;
    
    // Define vector of minimum thresholhds
    std::vector<float> dist_thresholds;

    // Calculate the vector of minimum thresholds
    for (int i = 0; i < peak_angles.size(); i++) {
        dist_thresholds.push_back((1 - ALPHA) * buf[peak_angles[i]]);
    }

    for (int j = 0; j < peak_angles.size(); j++) {
        // Left-side scan
        for (int k = peak_angles[j]; k >= peak_angles[j] - (min_sweep_angles[j] / 2); k--) {
            if (buf[k] < dist_thresholds[j]) {
                // Reset lt_count to 0, otherwise it will keep accumulating
                lt_count = 0;

                // Check 'num_checks' times to see if the adjacent points are also below the threshold distance
                for (unsigned int i = 0; i < num_checks; i++) {
                    // Lower clamp
                    if ((k - i) < 0) {
                        break;
                    }

                    if (buf[k - i] < dist_thresholds[j]) {
                        lt_count++;
                    } else {
                        // If we go back above the threshold, then the values are not consistent, so we go back to regular checking
                        break;
                    }
                }

                // If we loop through all the extra 'num_checks' points and all the points are less than the minimum threhsold, we can...
                // ... safely assume that we originally found an edge
                if (lt_count == num_checks) {
                    // Update values
                    // Record the distance measurement at which we cross over, and the angle at which this occurs
                    // Then break out of the for loop
                    // Bear in mind, I need the last good point, not the first bad point. This is why I use j + 1
                    if (k == size - 1) {
                        end_distance1 = buf[k];
                        end_angle1 = k;
                    } else {
                        end_distance1 = buf[k + 1];
                        end_angle1 = k + 1;
                    }
                    
                    // Update the flag to indicate that successive distance meausurements have dropped below the threshold distance
                    found_left = true;
                    
                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } 
        }

        // Right-side scan
        for (int k = peak_angles[j]; k <= peak_angles[j] + (min_sweep_angles[j] / 2); k++) {
            if (buf[k] < dist_thresholds[j]) {
                // Reset lt_count to 0, otherwise it will keep accumulating
                gt_count = 0;

                if ((peak_angles[j] + (min_sweep_angles[j] / 2)) >= size) {
                    break;
                }

                // Check 'num_checks' times to see if the adjacent points are also below the threshold distance
                for (unsigned int i = 0; i < num_checks; i++) {
                    if (k + i >= size) {
                        break;
                    }

                    if (buf[k + i] < dist_thresholds[j]) {
                        gt_count++;
                    } else {
                        // If we go back above the threshold, then the values are not consistent, so we go back to regular checking
                        break;
                    }
                }

                // If we loop through all the extra 'num_checks' points and all the points are less than the minimum threhsold, we can...
                // ... safely assume that we originally found an edge
                if (gt_count == num_checks) {
                    // Update values
                    // Record the distance measurement at which we cross over, and the angle at which this occurs
                    // Then break out of the for loop
                    // Bear in mind, I need the last good point, not the first bad point. This is why I use j + 1
                    end_angle2 = k - 1;
                    end_distance2 = buf[end_angle2];
                    
                    // Update the flag to indicate that successive distance meausurements have dropped below the threshold distance
                    found_right = true;

                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } 
        }

        if (!found_left) {
            end_angle1 = std::max(0, peak_angles[j] - (min_sweep_angles[j] / 2));
            end_distance1 = buf[end_angle1];
        }

        if (!found_right) {
            end_angle2 = std::min(size - 1, peak_angles[j] + (min_sweep_angles[j] / 2));
            end_distance2 = buf[end_angle2];
        }

        // Ok, we have done both sweeps. Now we calculate the gap width and store it in our vector
        float gap_width = calc_width(end_distance1, end_angle1, end_distance2, end_angle2);
        gap_widths.push_back(gap_width);

        // Ok, now we have finished calculating the gap width
        // Now, reset the variables for the next iteration
        end_distance1 = 0.0f;
        end_distance2 = 0.0f;
        end_angle1 = 0;
        end_angle2 = 0;

    }
    
    return gap_widths;
    
}

void Navigation::median_filter(uint16_t buf[], int size) {
    std::vector<uint16_t> temp(buf, buf + size);
    for (int i = 1; i < size - 1; i++) {
        uint16_t a = temp[i - 1], b = temp[i], c = temp[i + 1];
        // Sort 3 values and take the middle
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        buf[i] = b;
    }
}

std::vector<Gap> Navigation::find_gaps(uint16_t buf[], int size) {
    std::vector<Gap> gaps;

    // Sanitise buffer
    for (int i = 0; i < size; i++) {
        if (buf[i] == 0xFFFF || buf[i] > MAX_RANGE) {
            buf[i] = MAX_RANGE;
        }
    }

    // Smooth out single-point spikes before gap detection
    median_filter(buf, size);

    // Compute variance of buf[idx-VARIANCE_WINDOW .. idx+VARIANCE_WINDOW]
    auto local_variance = [&](int idx) -> float {
        int lo = std::max(0, idx - VARIANCE_WINDOW);
        int hi = std::min(size - 1, idx + VARIANCE_WINDOW);
        int n  = hi - lo + 1;

        float mean = 0.0f;
        for (int k = lo; k <= hi; k++) mean += buf[k];
        mean /= n;

        float var = 0.0f;
        for (int k = lo; k <= hi; k++) {
            float diff = (float)buf[k] - mean;
            var += diff * diff;
        }
        return var / n;
    };

    // A point is "open" if it exceeds MIN_GAP_DISTANCE AND
    // its drop from the previous point doesn't indicate an obstacle edge
    auto is_open = [&](int idx) -> bool {
        if (buf[idx] < MIN_GAP_DISTANCE) return false;
        if (idx > 0 && buf[idx - 1] > 0) {
            float prev = (float)buf[idx - 1];
            float curr = (float)buf[idx];
            float drop = (prev - curr) / prev;   // fractional drop
            if (drop > MAX_STEP_FRACTION) return false;
        }

        // Variance check - reject noisy regions even if distance is high
        if (local_variance(idx) > MAX_GAP_VARIANCE) return false;

        return true;
    };

    int i = 0;
    while (i < size) {
        // Find confirmed gap start: MIN_GAP_CONFIRM consecutive open points
        int confirm = 0;
        while (i < size && confirm < MIN_GAP_CONFIRM) {
            if (is_open(i)) {
                confirm++;
            } else {
                confirm = 0;
            }
            i++;
        }

        if (confirm < MIN_GAP_CONFIRM) break;

        Gap g;
        g.start_angle = i - MIN_GAP_CONFIRM;
        if (g.start_angle < 0) g.start_angle = 0;
        g.start_dist = buf[g.start_angle];

        // Find gap end: GAP_HYSTERESIS consecutive non-open points
        int below = 0;
        while (i < size) {
            if (!is_open(i)) {
                below++;
                if (below >= GAP_HYSTERESIS) break;
            } else {
                below = 0;
            }
            i++;
        }

        g.end_angle = i - GAP_HYSTERESIS;
        if (g.end_angle < 0)      g.end_angle = 0;
        if (g.end_angle >= size)  g.end_angle = size - 1;
        if (g.end_angle <= g.start_angle) continue;

        g.end_dist = buf[g.end_angle];
        g.width = calc_width(g.start_dist, g.start_angle, g.end_dist,   g.end_angle);

        if (g.width >= ROVER_WIDTH + SAFETY_MARGIN) {
            gaps.push_back(g);
        }
    }

    return gaps;
}

int Navigation::choose_direction(std::vector<float> gaps) {
    // Now find the largest gap width
    float largest_gw = 0;
    int index = -1;

    for (int i = 0; i < gaps.size(); i++) {
        if (gaps[i] > largest_gw) {
            largest_gw = gaps[i];
            index = i;
        }
    }

    return index;

}

// This function should simply skid-steer into place until the angle desired is equal to the change in yaw from the initial yaw 
// The arguments are the initial yaw (found by updating the IMU and measuring the yaw) and the final yaw (found by calculation based on the angle you desire)
void Navigation::skid_into_position(float start_yaw, float delta_deg) {
    while (delta_deg >  180.0f) delta_deg -= 360.0f;
    while (delta_deg <= -180.0f) delta_deg += 360.0f;

    if (fabsf(delta_deg) < SKID_DEADBAND_DEG) return;  // ignore tiny rotations

    if (delta_deg > 0.0f) {
        // clockwise
        dr_.skid_right(); 
    } else { 
        dr_.skid_left();   // anticlockwise
    }

    while (true) {
        imu_.update();
        ImuData data = imu_.read();

        if (!data.valid) { 
            sleep_ms(10); 
            continue; 
        }

        // Each time, we measure the yaw from the IMU and compare to the start_yaw until the change in yaw is the change we need
        float actual_delta = data.yaw_deg - start_yaw;

        // Wrap actual_delta to (-180, +180] to handle the 0°/360° boundary
        while (actual_delta > 180.0f) {
            actual_delta -= 360.0f;
        }

        while (actual_delta <= -180.0f) {
            actual_delta += 360.0f;
        }

        bool done = false;
        if (delta_deg > 0.0f && actual_delta >=  SKID_STOP_FRACTION * delta_deg) {
            done = true;
        }

        if (delta_deg < 0.0f && actual_delta <=  SKID_STOP_FRACTION * delta_deg) {
            done = true;
        }

        if (done) { 
            dr_.brake(); 
            break; 
        }

        // Add small delay to reduce I2C spam
        sleep_ms(10);

    }

}

void Navigation::reset_buffer(uint16_t lidar_buff[], int size) {
    for (int i = 0; i < size; i++) {
        lidar_buff[i] = 0;
    }
} 

bool Navigation::space_check(uint16_t buf[], int size) {
    for (int i = 0; i < size; i++) {
        if (buf[i] == MAX_RANGE) {
            return true;
        }
    }

    return false;
}

int Navigation::check_max_range(uint16_t buf[], int size) {
    int best_count = 0;
    int best_start = 0;
    int count = 0;
    int cur_start = 0;

    for (int i = 0; i < size; i++) {           
        if (buf[i] >= MAX_RANGE) {
            if (count == 0) cur_start = i;     
            count++;
            if (count > best_count) {           
                best_count = count;
                best_start = cur_start;
            }
        } else {
            count = 0;                        
        }
    }
   
    if (best_count == 0) {
        return size / 2;
    }

    if (best_count % 2 == 0) {
        return (best_count / 2) + best_start;
    } else {
        return ((best_count + 1) / 2) + best_start;
    }
}

std::vector<int> Navigation::detect_edge(uint16_t buf[], int size) {
    std::vector<int> edge_angles;

    if (size < 3) return edge_angles;  // not enough points for i-1, i, i+1
    if (size < 2 * NUM_CHECKS + 1) return edge_angles; // need enough points for checks

    for (int i = 1; i < size - 1; i++) {

        // local minimum candidate
        if ((buf[i - 1] > buf[i]) && (buf[i + 1] > buf[i])) {

            int left_count = 0;
            int right_count = 0;

            bool left_flag = false;
            bool right_flag = false;

            // Check left side safely
            for (int k = 1; k <= NUM_CHECKS; k++) {
                if (i - k < 0) break;  // stop if we'd go negative

                if (buf[i - k] > buf[i]) {
                    left_count++;
                } else {
                    left_count = 0;
                    break;
                }
            }

            if (left_count == NUM_CHECKS) {
                left_flag = true;
            }

            // Check right side safely
            for (int j = 1; j <= NUM_CHECKS; j++) {
                if (i + j >= size) break;  // stop if we'd exceed array

                if (buf[i + j] > buf[i]) {
                    right_count++;
                } else {
                    right_count = 0;
                    break;
                }
            }

            if (right_count == NUM_CHECKS) {
                right_flag = true;
            }

            if (left_flag && right_flag) {
                edge_angles.push_back(i);
            }
        }
    }

    return edge_angles;
}

bool Navigation::check_openings(uint16_t buf[], int size, int min_dist) {
    for (int i = 0; i  < size; i++) {
        if (buf[i] < min_dist) {
            return false;
        }
    }

    return true;
}

