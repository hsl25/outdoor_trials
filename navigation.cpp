#include <stdio.h>
#include <math.h>
#include <vector>

#include "navigation.hpp"
#include "servo.hpp"
#include "IMU.hpp"
#include "driving.hpp"

TOF my_tof;
Servo sv;
IMU my_imu(i2c0, IMU_SDA_PIN, IMU_SCL_PIN, MPU6050_ADDRESS_A0_GND);
Drive dr;

// Constructor
Navigation::Navigation() {
    
}

// Start continuous ranging
// This function still needs to be modified for adding data into a buffer and calibrating
// This is how calibration will run:
// 1. The servo will start at position 0 degrees
// 2. The servo will increment 1 degree. It will then indicate via one of something like: toggling a bit, raising a flag, etc. 
// 3. The LiDAR will then check if data is available, and if it is available, it will add it to the correct buffer position
// 4. After data has been successfully added, the servo will increment the angle again and the process will start again

// This function takes as input the lidar buffer and its size, and also the number of times we will sweep back and forth
// It simply performs the sweeping and loads data into the buffer
void Navigation::forward_sweep(int num_sweeps, uint16_t lidar_buf[], int size) {
    // Keep track of the number of sweeps the servo has done
    int num = 0;

    std::vector<uint32_t> accum(size, 0);

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num < num_sweeps) {
        for (int i = 0; i < size; i++) {
            // Now increment the angle of the servo by 1 degree
            sv.set_front_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(100); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = my_tof.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            accum[i] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        for (int i = size - 1; i >= 0; i--) {
            // Now increment the angle of the servo by 1 degree
            sv.set_front_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(100); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = my_tof.read_tof_continuous();
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS);
            accum[i] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer 
            // buffer.add_calib_sample(temp, i);  

        }
        
        num++;

    }

    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (accum[i] / (2 * num_sweeps));
    }

    // For debugging
    printf("Front sweep complete.\n");

}

// This function takes as input the lidar buffer and its size, and also the number of times we will sweep back and forth
// It simply performs the sweeping and loads data into the buffer
void Navigation::rear_sweep(int num_sweeps, uint16_t lidar_buf[], int size) {
    // Keep track of the number of sweeps the servo has done
    int num = 0;

    std::vector<uint32_t> accum(size, 0);

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num < num_sweeps) {
        for (int i = 0; i < size; i++) {
            // Now increment the angle of the servo by 1 degree
            sv.set_rear_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(100); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = my_tof.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            accum[i] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        for (int i = size - 1; i >= 0; i--) {
            // Now increment the angle of the servo by 1 degree
            sv.set_rear_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(100); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = my_tof.read_tof_continuous();
            // float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS);
            accum[i] += lidar_data;
            // lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer 
            // buffer.add_calib_sample(temp, i);  

        }
        
        num++;

    }

    for (int i = 0; i < size; i++) {
        lidar_buf[i] = (accum[i] / (2 * num_sweeps));
    }

    // For debugging
    printf("Rear sweep complete.\n");

}

void Navigation::print_buffer(uint16_t buf[], int size) {
    for (int i = 0; i < size; i++) {
        printf("Angle: %d   Distance(mm): %u \n", i, buf[i]);
    }
}

float Navigation::calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2) {
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
                    
                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } else if (buf[k] > dist_thresholds[j]) {
                // We just do nothing because we want the values to be greater than the minimum threshold
                continue;
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
                    end_distance2 = buf[k - 1];
                    end_angle2 = k - 1;
                    // Now we need to break out of the outer for loop, since the edge has been detected
                    break;
                }
            } else if (buf[k] > dist_thresholds[j]) {
                // We just do nothing because we want the values to be greater than the minimum threshold
                continue;
            }
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
void Navigation::skid_into_position(float start_yaw, float final_yaw) {
    float delta = 0.0f;
    int steer_angle = (int) (final_yaw - start_yaw);

    if (steer_angle > 0) {
            // Skid-steer left (or anticlockwise to be more specific) until the yaw matches 20 degrees
            dr.skid_left();

            while (true) {
                my_imu.update();
                ImuData data = my_imu.read();

                // Check if the data is valid, and if so, store the yaw angle as the current yaw angle
                if (data.valid) {
                    // Each time, we measure the yaw from the IMU and compare to the start_yaw until the change in yaw is the change we need
                    delta = data.yaw_deg - start_yaw;

                    // If delta >= steer_angle, then the change in angle w.r.t start_yaw is the same as or greater than the change we require, so we brake and break
                    if (delta >= steer_angle) {
                        dr.brake();
                        break;
                    }
                } else {
                    printf("IMU data invalid\r\n");
                }

                // Add small delay to reduce I2C spam
                sleep_ms(10);

            }

        } else if (steer_angle < 0) {
            dr.skid_right();

            while (true) {
                my_imu.update();
                ImuData data = my_imu.read();

                // Check if the data is valid, and if so, store the yaw angle as the current yaw angle
                if (data.valid) {
                    // Each time, we measure the yaw from the IMU and compare to the start_yaw until the change in yaw is the change we need
                    delta = data.yaw_deg - start_yaw;

                    // If delta >= steer_angle, then the change in angle w.r.t start_yaw is the same as or greater than the change we require, so we brake and break
                    if (delta <= steer_angle) {
                        dr.brake();
                        break;
                    }
                } else {
                    printf("IMU data invalid\r\n");
                }

                // Add small delay to reduce I2C spam
                sleep_ms(10);

            }

        } else if (steer_angle == 0) {
            // More to be done here later
            dr.drive_forward();
        }
}

void Navigation::reset_buffer(uint16_t lidar_buff[], int size) {
    for (int i = 0; i < size; i++) {
        lidar_buff[i] = 0;
    }
} 


