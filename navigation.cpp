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
        for (int i = SERVO_MIN_SWEEP_ANGLE; i <= SERVO_MAX_SWEEP_ANGLE; i += ANGLE_STEP) {
            // Now increment the angle of the servo by 1 degree
            sv_.set_front_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(10); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = tof_.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            accum[(i - SERVO_MIN_SWEEP_ANGLE) / ANGLE_STEP] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        // Return servo to start position ready for next sweep
        sv_.set_front_angle(SERVO_MIN_SWEEP_ANGLE);
        sleep_ms(300);
        
        num++;

    }

    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (uint16_t)(accum[i] / num_sweeps);
    }

    // For debugging
    // printf("Front sweep complete.\n");

}

// This function takes as input the lidar buffer and its size, and also the number of times we will sweep back and forth
// It simply performs the sweeping and loads data into the buffer
void Navigation::rear_sweep(int num_sweeps, uint16_t lidar_buf[], int size) {
    int num = 0;
    std::vector<uint32_t> accum(size, 0);
 
    while (num < num_sweeps) {
        for (int i = SERVO_MIN_SWEEP_ANGLE; i <= SERVO_MAX_SWEEP_ANGLE; i += ANGLE_STEP) {
            sv_.set_rear_angle(i);
            sleep_ms(10);
            uint16_t lidar_data = tof_.read_tof_continuous();
            accum[(i - SERVO_MIN_SWEEP_ANGLE) / ANGLE_STEP] += lidar_data;
        }
 
        sv_.set_rear_angle(SERVO_MIN_SWEEP_ANGLE);
        sleep_ms(300);
        num++;
    }
 
    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (uint16_t)(accum[i] / num_sweeps);
    }
}

void Navigation::print_buffer(uint16_t buf[], int size) {
    for (int i = 0; i < size; i++) {
        // Each buffer index i represents ANGLE_STEP degrees per slot
        int angle_to_print = SERVO_MIN_SWEEP_ANGLE + (i * ANGLE_STEP);
        printf("Angle: %d   Distance(mm): %d\n", angle_to_print, (int)buf[i]);
    }
}

float Navigation::calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2) {
    int theta_diff = (angle2 - angle1) * ANGLE_STEP;  // convert index diff to degrees
    float x1 = (float)length1;
    float x2 = (float)length2;
    return sqrtf((x1*x1) + (x2*x2) - (2.0f * x1 * x2 * cosf(theta_diff * DEG_TO_RAD)));
}

std::vector<int> Navigation::calc_peaks(uint16_t arr[], int size) {
    std::vector<int> peaks;
    for (int i = 1; i < size - 1; i++) {
        if (arr[i] > arr[i-1] && arr[i] > arr[i+1]) {
            peaks.push_back(i);
        }
    }
    return peaks;
}

int Navigation::calc_min_sweep_angle(float dist) {
    float temp_angle = 2.0f * atanf((ROVER_WIDTH + SAFETY_MARGIN) / (2.0f * dist));
    temp_angle *= RAD_TO_DEG;
    // Round up to next even number of degrees
    int min_deg = (int)(temp_angle + 1.0f);
    if (min_deg % 2 != 0) min_deg++;
    // Convert degrees to buffer index units
    return (min_deg + ANGLE_STEP - 1) / ANGLE_STEP;  // ceiling division
}

void Navigation::median_filter(uint16_t buf[], int size) {
    std::vector<uint16_t> temp(buf, buf + size);
    for (int i = 1; i < size - 1; i++) {
        uint16_t a = temp[i-1], b = temp[i], c = temp[i+1];
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        buf[i] = b;
    }
}

// Arguments needed:
// - the vector of peak angles 
// - the vector of minimum sweep angles 
// - the entire LiDAR buffer and its size 
std::vector<float> Navigation::calc_gap_width(std::vector<int> peak_angles,
                                               std::vector<int> min_sweep_angles,
                                               uint16_t buf[], int size) {
    std::vector<float> gap_widths;
    int num_checks = 3;
    int lt_count = 0, gt_count = 0;
    float end_distance1 = 0.0f, end_distance2 = 0.0f;
    int end_angle1 = 0, end_angle2 = 0;
    bool found_left = false, found_right = false;
 
    std::vector<float> dist_thresholds;
    for (int i = 0; i < (int)peak_angles.size(); i++) {
        dist_thresholds.push_back((1.0f - ALPHA) * buf[peak_angles[i]]);
    }
 
    for (int j = 0; j < (int)peak_angles.size(); j++) {
        found_left = false; found_right = false;
 
        for (int k = peak_angles[j]; k >= peak_angles[j] - (min_sweep_angles[j] / 2); k--) {
            if (k < 0) break;
            if (buf[k] < dist_thresholds[j]) {
                lt_count = 0;
                for (int ii = 0; ii < num_checks; ii++) {
                    if (k - ii < 0) break;
                    if (buf[k - ii] < dist_thresholds[j]) lt_count++;
                    else break;
                }
                if (lt_count == num_checks) {
                    end_distance1 = buf[std::min(k + 1, size - 1)];
                    end_angle1    = std::min(k + 1, size - 1);
                    found_left = true;
                    break;
                }
            }
        }
 
        for (int k = peak_angles[j]; k <= peak_angles[j] + (min_sweep_angles[j] / 2); k++) {
            if (k >= size) break;
            if (buf[k] < dist_thresholds[j]) {
                gt_count = 0;
                for (int ii = 0; ii < num_checks; ii++) {
                    if (k + ii >= size) break;
                    if (buf[k + ii] < dist_thresholds[j]) gt_count++;
                    else break;
                }
                if (gt_count == num_checks) {
                    end_angle2    = std::max(k - 1, 0);
                    end_distance2 = buf[end_angle2];
                    found_right = true;
                    break;
                }
            }
        }
 
        if (!found_left) {
            end_angle1    = std::max(0, peak_angles[j] - (min_sweep_angles[j] / 2));
            end_distance1 = buf[end_angle1];
        }
        if (!found_right) {
            end_angle2    = std::min(size - 1, peak_angles[j] + (min_sweep_angles[j] / 2));
            end_distance2 = buf[end_angle2];
        }
 
        gap_widths.push_back(calc_width((uint16_t)end_distance1, end_angle1, (uint16_t)end_distance2, end_angle2));
        end_distance1 = end_distance2 = 0.0f;
        end_angle1 = end_angle2 = 0;
    }
 
    return gap_widths;
}

std::vector<Gap> Navigation::find_gaps(uint16_t buf[], int size) {
    std::vector<Gap> gaps;
 
    // Sanitise
    for (int i = 0; i < size; i++) {
        if (buf[i] == 0xFFFF || buf[i] > MAX_RANGE) buf[i] = MAX_RANGE;
    }
 
    median_filter(buf, size);
 
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
 
    auto is_open = [&](int idx) -> bool {
        if (buf[idx] < MIN_GAP_DISTANCE) return false;
        if (idx > 0 && buf[idx-1] > 0) {
            float prev = (float)buf[idx-1];
            float curr = (float)buf[idx];
            float drop = (prev - curr) / prev;
            if (drop > MAX_STEP_FRACTION) return false;
        }
        if (local_variance(idx) > MAX_GAP_VARIANCE) return false;
        return true;
    };
 
    int i = 0;
    while (i < size) {
        int confirm = 0;
        while (i < size && confirm < MIN_GAP_CONFIRM) {
            if (is_open(i)) confirm++;
            else confirm = 0;
            i++;
        }
        if (confirm < MIN_GAP_CONFIRM) break;
 
        Gap g;
        g.start_angle = std::max(0, i - MIN_GAP_CONFIRM);
        g.start_dist  = buf[g.start_angle];
 
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
        if (g.end_angle < 0)     g.end_angle = 0;
        if (g.end_angle >= size) g.end_angle = size - 1;
        if (g.end_angle <= g.start_angle) continue;
 
        g.end_dist = buf[g.end_angle];
        g.width    = calc_width(g.start_dist, g.start_angle, g.end_dist, g.end_angle);
 
        if (g.width >= (float)(ROVER_WIDTH + SAFETY_MARGIN)) {
            gaps.push_back(g);
        }
    }
 
    return gaps;
}

int Navigation::choose_direction(std::vector<float> gaps) {
    float largest = 0.0f;
    int index = -1;
    for (int i = 0; i < (int)gaps.size(); i++) {
        if (gaps[i] > largest) { largest = gaps[i]; index = i; }
    }
    return index;
}

// This function should simply skid-steer into place until the angle desired is equal to the change in yaw from the initial yaw 
// The arguments are the initial yaw (found by updating the IMU and measuring the yaw) and the final yaw (found by calculation based on the angle you desire)
void Navigation::skid_into_position(float delta_deg) {
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }

    while (delta_deg <= -180.0f) {
        delta_deg += 360.0f;
    }

    if (fabsf(delta_deg) < SKID_DEADBAND_DEG) {
        return;
    }

    float skid_time_s = dr_.calc_skid_time(delta_deg);
    uint32_t skid_ms = (uint32_t)(skid_time_s * 1000.0f);

    printf("skid: delta=%.1f deg, time=%lu ms\n", delta_deg, skid_ms);

    if (delta_deg > 0.0f) { 
        dr_.skid_right();
    } else {
        dr_.skid_left();
    }                  

    sleep_ms(skid_ms);
    dr_.brake();
}

void Navigation::reset_buffer(uint16_t lidar_buff[], int size) {
    for (int i = 0; i < size; i++) lidar_buff[i] = 0;
}

bool Navigation::space_check(uint16_t buf[], int size) {
    for (int i = 0; i < size; i++) {
        if (buf[i] == MAX_RANGE) return true;
    }
    return false;
}

int Navigation::check_max_range(uint16_t buf[], int size, int dist_choice) {
    int best_count = 0, best_start = 0;
    int count = 0, cur_start = 0;
 
    for (int i = 0; i < size; i++) {
        if (buf[i] >= dist_choice) {
            if (count == 0) cur_start = i;
            count++;
            if (count > best_count) { best_count = count; best_start = cur_start; }
        } else {
            count = 0;
        }
    }
 
    if (best_count == 0) return size / 2;
    return best_start + best_count / 2;
}

std::vector<int> Navigation::detect_edge(uint16_t buf[], int size) {
    std::vector<int> edge_angles;
 
    if (size < 3) return edge_angles;
    if (size < 2 * NUM_CHECKS + 1) return edge_angles;
 
    for (int i = 1; i < size - 1; i++) {
        if (buf[i-1] > buf[i] && buf[i+1] > buf[i]) {
            int left_count = 0, right_count = 0;
 
            for (int k = 1; k <= NUM_CHECKS; k++) {
                if (i - k < 0) break;
                if (buf[i-k] > buf[i]) left_count++;
                else { left_count = 0; break; }
            }
 
            for (int j = 1; j <= NUM_CHECKS; j++) {
                if (i + j >= size) break;
                if (buf[i+j] > buf[i]) right_count++;
                else { right_count = 0; break; }
            }
 
            if (left_count == NUM_CHECKS && right_count == NUM_CHECKS) {
                edge_angles.push_back(i);
            }
        }
    }
 
    return edge_angles;
}

bool Navigation::check_openings(uint16_t buf[], int size, int min_dist) {
    for (int i = 0; i < size; i++) {
        if (buf[i] < min_dist) return false;
    }
    return true;
}

int Navigation::max_index(uint16_t buf[], int size) {
    int best = 0;
    for (int i = 0; i < size; i++) {
        if (buf[i] > buf[best]) best = i;
    }
    return best;
}

BestGap Navigation::find_best_gap(uint16_t buf[], int size) {
    BestGap result = {false, 0, 0.0f, 0.0f, 0.0f};
 
    std::vector<Gap> gaps = find_gaps(buf, size);
    if (gaps.empty()) return result;
 
    int best = 0;
    for (int i = 1; i < (int)gaps.size(); i++) {
        if (gaps[i].width > gaps[best].width) best = i;
    }
 
    Gap& g = gaps[best];
    int centre = (g.start_angle + g.end_angle) / 2;
 
    result.valid          = true;
    result.centre_index   = centre;
    result.width_mm       = g.width;
    result.centre_dist_mm = (float)buf[centre];
    // Convert buffer index to IMU heading delta:
    //   absolute angle = SERVO_MIN_SWEEP_ANGLE + centre * ANGLE_STEP
    //   delta from straight-ahead (90 deg) = absolute angle - 90
    result.imu_angle_deg  = (float)(SERVO_MIN_SWEEP_ANGLE + centre * ANGLE_STEP - 90);
 
    return result;
}

