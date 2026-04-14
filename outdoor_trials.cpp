#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include <vector>

// File includes
#include "buffer.hpp"
#include "driving.hpp"
#include "tof.hpp"
#include "servo.hpp"
#include "IMU.hpp"
#include "navigation.hpp"

// Defines
#define ROVER_WIDTH 600 // Width of the rover in mm
#define ROVER_LENGTH 440 // Length of rover in mm
#define SAFETY_MARGIN 10 // 10mm safety margin added to the width and length of the rover
#define ALPHA 0.2 // Safety constant

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

    // Calculate the minimum angle needed to see if the gap is large enough 
    // atan = arctan 
    float temp_angle = 2 * atan((ROVER_WIDTH + SAFETY_MARGIN) / (2 * largest_distance)); 

    // Now I need to round up - always round up because if the minimum angle is 30.2 degrees, it is safer to have a larger angle 
    // I can do this by adding 1 and then casting to an int, so the decimal places will be 'chopped off' 
    // I also need to ensure that the minimum angle is even so that I have an even number angle either side of... 
    // ... the 'greatest_distance_angle' 
    int min_sweep_angle = (int) (temp_angle + 1); 

    if (min_sweep_angle % 2 != 0) { 
        min_sweep_angle++; 
    } 

    
    
    float beta1 = 0.0f; // Rolling value for the 1st scan
    int tracker1 = 0; // Used to keep track of the last valid value
    float roll_mean1 = 0.0f;
    float end_distance1 = 0.0f;
    int end_angle1 = 0;

    // Now scan through the window we just calculated 
    for (int j = largest_clearance_angle; j >= largest_clearance_angle - (min_sweep_angle / 2); j--) { 
        if (j == largest_clearance_angle) {
            roll_mean1 = lidar_buffer[j];
        }

        // At least two points are needed to calculate an initial value for beta 
        if (j == largest_clearance_angle - 1) {
            // Calculate beta. Beta is the ratio between consecutive points. It is used to identify whether or not the next point is a big jump or not
            beta1 = lidar_buffer[largest_clearance_angle - 1] / lidar_buffer[largest_clearance_angle];
            tracker1 = largest_clearance_angle - 1;
        }

        // Check if the next measurement we look at in the buffer lies in the inequality
        // If it does, update the rolling mean and the beta value
        if ((roll_mean1 - (beta1 * ALPHA) <= lidar_buffer[j]) && (lidar_buffer[j] <= roll_mean1 + (beta1 * ALPHA))) {
            // If the next point falls into the inequality, update the rolling mean
            roll_mean1 += lidar_buffer[j];
            // Also update beta 
            beta1 += lidar_buffer[j] / lidar_buffer[tracker1];
            // Update tracker1
            tracker1 = j; 

        } else {
            // If the value does not fall into the inequality, then it could be a large jump
            // To check for large jumps, we need to check for consistency, so start a new set of statistics and see if the values are consistent
            // We check for consistency because the value could be a glitch, or could be a very small thing
            end_distance1 = lidar_buffer[j];
            end_angle1 = j;
        }

    } 

    float beta2 = 0.0f; // Rolling value for the 2nd scan
    int tracker2 = 0; // Used to keep track of the last valid value
    float roll_mean2 = 0.0f;
    float end_distance2 = 0.0f;
    int end_angle2 = 0;

    // Now scan the other way
    for (int k = largest_clearance_angle; k <= largest_clearance_angle + (min_sweep_angle / 2); k++) { 
        if (k == largest_clearance_angle) {
            roll_mean2 = lidar_buffer[k];
        }

        // At least two points are needed to calculate an initial value for beta 
        if (k == largest_clearance_angle + 1) {
            // Calculate beta. Beta is the ratio between consecutive points. It is used to identify whether or not the next point is a big jump or not
            beta2 = lidar_buffer[largest_clearance_angle + 1] / lidar_buffer[largest_clearance_angle];
            tracker2 = largest_clearance_angle + 1;
        }

        // Check if the next measurement we look at in the buffer lies in the inequality
        // If it does, update the rolling mean and the beta value
        if ((roll_mean2 - (beta2 * ALPHA) <= lidar_buffer[k]) && (lidar_buffer[k] <= roll_mean2 + (beta2 * ALPHA))) {
            // If the next point falls into the inequality, update the rolling mean
            roll_mean2 += lidar_buffer[k];
            // Also update beta 
            beta2 += lidar_buffer[k] / lidar_buffer[tracker2];
            // Update tracker1
            tracker2 = k; 

        } else {
            // If the value does not fall into the inequality, then it could be a large jump
            // To check for large jumps, we need to check for consistency, so start a new set of statistics and see if the values are consistent
            // We check for consistency because the value could be a glitch, or could be a very small thing
            end_distance2 = lidar_buffer[k];
            end_angle2 = k;
        }

    }

    float gap_width = nav.calc_width(end_distance1, end_angle1, end_distance2, end_angle2);

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


