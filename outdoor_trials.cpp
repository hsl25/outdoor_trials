#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <vector>
#include <algorithm>

// File includes
#include "buffer.hpp"
#include "driving.hpp"
#include "tof.hpp"
#include "servo.hpp"
#include "IMU.hpp"
#include "navigation.hpp"

// Instantiate objects
TOF tof;
Drive drive;
Buffer buffer;
Servo servo;
IMU imu(i2c0, IMU_SDA_PIN, IMU_SCL_PIN, MPU6050_ADDRESS_A0_GND);
Navigation nav;

uint16_t lidar_buffer[MAX_SERVO_ANGLE + 1];

int main() {
    // Initialise serial monitor just in case I need it for debugging 
    stdio_init_all();

    // Initialise I2C and UART
    tof.init_i2c();
    tof.init_uart();

    // Setting up the VL53L0X LiDAR and checking for errors
    tof.device_setup();

    // Initialise IMU
    imu.init();

    // Initialise motors
    drive.init_pwm_mode();
    drive.init_clk_divider();
    drive.setup_motors();

    // TOF calibration
    // This involves checking for errors and setting up timing budget
    tof.calibration();

    // Start continuous ranging
    // This function still needs to be modified for adding data into a buffer and calibrating
    // This is how calibration will run:
    // 1. The servo will start at position 0 degrees
    // 2. The servo will increment 1 degree. It will then indicate via one of something like: toggling a bit, raising a flag, etc. 
    // 3. The LiDAR will then check if data is available, and if it is available, it will add it to the correct buffer position
    // 4. After data has been successfully added, the servo will increment the angle again and the process will start again

    // Keep track of the number of sweeps the servo has done
    int num_sweeps = 0;

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num_sweeps < CALIBRATION_SWEEPS) {
        for (int i = 0; i <= MAX_SERVO_ANGLE; i++) {
            // Now increment the angle of the servo by 1 degree
            servo.set_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(15); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = tof.read_tof_continuous();
            // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
            // I define 1 sweep as 0 -> 180 and 180 -> 0
            float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
            lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer
            // buffer.add_calib_sample(temp, i); 
  
        }

        for (int i = MAX_SERVO_ANGLE; i >= 0; i--) {
            // Now increment the angle of the servo by 1 degree
            servo.set_angle(i);

            // Wait for servo to physically reach the degree
            sleep_ms(15); 

            // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
            // Then, the data point is added to the buffer
            // This is step 3
            uint16_t lidar_data = tof.read_tof_continuous();
            float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS);
            lidar_buffer[i] += (uint16_t) temp;
            // // Instead of making a function to add a sample, do it normally for any buffer 
            // buffer.add_calib_sample(temp, i);  

        }
        
        num_sweeps++;

    }

    // For debugging
    printf("Rover calibration complete.\n");

    // Identify all the angles at which peaks occur in the buffer
    std::vector<int> peak_angles = nav.calc_peaks(lidar_buffer, MAX_SERVO_ANGLE + 1);

    // OK so now the rover has to identify the direction of greatest clearance 
    // This is simply looping through the buffer and identifying the index with the greatest clearance  
    int largest_clearance_angle = 0; 
    uint16_t largest_distance = 0; 

    for (int i = 1; i <= MAX_SERVO_ANGLE; i++) { 
        if (lidar_buffer[i] > largest_distance) { 
            largest_clearance_angle = i; 
            largest_distance = lidar_buffer[i]; 
        } 
    } 

    std::vector<int> min_sweep_angles;

    for (int x = 0; x < peak_angles.size(); x++) {
        // Calculate the minimum sweep angle for each peak calculated before
        int msa = nav.calc_min_sweep_angle(lidar_buffer[peak_angles[x]]);
        min_sweep_angles.push_back(msa);
    }

    // Calculate the gap widths for each of the peaks
    std::vector<float> gap_widths = nav.calc_gap_width(peak_angles, min_sweep_angles, lidar_buffer, MAX_SERVO_ANGLE + 1);

    // Find the largest gap out of all the calculated gaps
    float largest_gap = nav.choose_direction(gap_widths);

    // Now I need to track back and find the peak responsible for this gap
    float chosen_peak = nav.find_peak(largest_gap, gap_widths);
    int chosen_angle = nav.find_angle(lidar_buffer, MAX_SERVO_ANGLE + 1, chosen_peak);

    // Implementing a function to enable the rover to skid-steer and face a specific angle 
    int test_angle = 20;
    float yaw_angle = 0.0f;
    
    imu.update();
    float start_yaw = imu.read().yaw_deg;
    float delta = 0.0f;

    if (test_angle > 0) {
        // Skid-steer left (or anticlockwise to be more specific) until the yaw matches 20 degrees
        drive.skid_left();

        while (true) {
            imu.update();
            ImuData data = imu.read();

            // Check if the data is valid, and if so, store the yaw angle as the current yaw angle
            if (data.valid) {
                delta = data.yaw_deg - start_yaw;

                if (delta >= test_angle) {
                    drive.brake();
                    break;
                }
            } else {
                printf("IMU data invalid\r\n");
            }

            // Add small delay to reduce I2C spam
            sleep_ms(10);

        }

    } else if (test_angle < 0) {
        drive.skid_right();

        while (true) {
            imu.update();
            ImuData data = imu.read();

            // Check if the data is valid, and if so, store the yaw angle as the current yaw angle
            if (data.valid) {
                delta = data.yaw_deg - start_yaw;

                if (delta <= test_angle) {
                    drive.brake();
                    break;
                }
            } else {
                printf("IMU data invalid\r\n");
            }

            // Add small delay to reduce I2C spam
            sleep_ms(10);

        }

    } else if (test_angle == 0) {
        // More to be done here later
        drive.drive_forward();
    }
    
    return 0;
}


