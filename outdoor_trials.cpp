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

uint32_t accum[MAX_SERVO_ANGLE + 1] = {0};
uint16_t lidar_buffer[MAX_SERVO_ANGLE + 1] = {0};

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

    while (1) {
        printf("1 \n");
    }

    // Keep track of the number of sweeps the servo has done
    int num_sweeps = 0;
    int temp = 360 / SKID_CHECK_ANGLE;

    // Establish variables
    float chosen_distance = 0.0f;
    int chosen_angle = 0;
    int run = 0;

    // Servo testing
    while (1) {
        sleep_ms(1000);
        servo.set_front_angle(0);
        sleep_ms(1000);
        servo.set_front_angle(90);
        sleep_ms(1000);
        servo.set_front_angle(180);
        sleep_ms(1000);
        servo.set_front_angle(90);
        sleep_ms(1000);
        servo.set_front_angle(0);
        sleep_ms(1000);
        printf("yes\n");
    }

    // Testing the lidar scanning
    // Scan the surroundings and measure distances
    nav.forward_sweep(CALIBRATION_SWEEPS, lidar_buffer, MAX_SERVO_ANGLE + 1);

    // Printing the entire buffer to know what the lidar reads
    nav.print_buffer(lidar_buffer, MAX_SERVO_ANGLE + 1);

    // while (1) {
    //     for (run = 0; run < temp; run++) {
    //         // Scan the surroundings and measure distances
    //         nav.initial_sweep(CALIBRATION_SWEEPS, lidar_buffer, MAX_SERVO_ANGLE + 1);

    //         // Identify all the angles at which peaks occur in the buffer
    //         std::vector<int> peak_angles = nav.calc_peaks(lidar_buffer, MAX_SERVO_ANGLE + 1);

    //         // Make a vector of all the sweep angles at the different peaks
    //         std::vector<int> min_sweep_angles;

    //         for (int x = 0; x < peak_angles.size(); x++) {
    //             // Calculate the minimum sweep angle for each peak calculated before
    //             int msa = nav.calc_min_sweep_angle(lidar_buffer[peak_angles[x]]);
    //             min_sweep_angles.push_back(msa);
    //         }

    //         // Calculate the gap widths for each of the peaks
    //         std::vector<float> gap_widths = nav.calc_gap_width(peak_angles, min_sweep_angles, lidar_buffer, MAX_SERVO_ANGLE + 1);

    //         // Find the largest gap out of all the calculated gaps
    //         // This returns the index of the largest gap
    //         int largest_gap_index = nav.choose_direction(gap_widths);

    //         // Now I need to track back and find the peak responsible for this gap
    //         // The values for gap_widths correspond to the peaks in peak_angles
    //         int chosen_angle = peak_angles[largest_gap_index];

    //         // Extract the distance from the lidar buffer using the angle index 
    //         chosen_distance = lidar_buffer[chosen_angle];

    //         if (chosen_distance > ROVER_WIDTH + SAFETY_MARGIN) {
    //             // Implementing a function to enable the rover to skid-steer and face a specific angle 
    //             // int test_angle = 20;
    //             float yaw_angle = 0.0f;
                
    //             imu.update();
    //             float start_yaw = imu.read().yaw_deg;

    //             // A positive yaw indicates a clockwise turn 
    //             // A negaive yaw indicates an anticlockwise turn
    //             // The current angle interpretation is 0 degrees at the left-most angle, 0 degrees at the normal, and 180 degress at the right-most angle
    //             // The IMU interpretation is -90 degrees at the left-most angle, 0 degrees at the normal, and +90 degrees at the right-most angle
    //             // Therefore, this is the mapping of servo angle --> IMU yaw angle
    //             // 0 degrees --> -90 degrees
    //             // 90 degrees --> 0 degrees
    //             // 180 degrees --> +90 degrees
    //             // Therefore, I need to subtract 90 degrees form 'chosen_angle'

    //             int imu_angle = chosen_angle - 90;
    //             int final_yaw = start_yaw + imu_angle;

    //             // You need to input the starting yaw and final yaw 
    //             // For example, say the chosen angle is a rotation by servo angle of 30 degrees
    //             // Then the IMU-based angle would be a rotation of -60 degrees. This is the 'target angle'
    //             // Let's say the starting yaw is +5 degrees
    //             // The final yaw would be -55 degrees if we rotate by -60 degrees
    //             // This would require skid-steering left (anticlockwise) since the change in yaw required is negative
    //             nav.skid_into_position(start_yaw, imu_angle);

    //             // Ok now we are in position so we can break and start navigating
    //             break;

    //         } else if (chosen_distance <= ROVER_WIDTH + SAFETY_MARGIN) {
    //             // Ok so there is no gap in front of us that is large enough
    //             // Now we need to skid-steer by 90 degrees and start the entire process again, all the way from scanning and relative localisation
    //             imu.update(); 
    //             float current_yaw = imu.read().yaw_deg;
    //             nav.skid_into_position(current_yaw, current_yaw + SKID_CHECK_ANGLE);

    //             // Reset variables
    //             num_sweeps = 0;
    //             chosen_angle = 0;
    //             chosen_distance = 0;

    //             // Reset buffer
    //             nav.reset_buffer(lidar_buffer, MAX_SERVO_ANGLE + 1);
                
    //         }

    //     }

    //     // OK so we know where to go, now we have to calculate how far to travel and also verify with the IMU
    //     float drive_time = drive.calc_drive_time(WHEEL_DIAMETER, OLD_MOTOR_RPM, chosen_distance / DISTANCE_DIVIDER);

    //     // Now drive
    //     drive.drive_forward();
    //     sleep_ms(drive_time);
    //     drive.brake();

    //     // Ok, now we have finished navigating. 
    //     // Now we simply scan again and navigate again, and this cycle continues till the rover is turned off via the GUI
    //     // We still need to reset everything though

    //     // Reset variables
    //     num_sweeps = 0;
    //     chosen_angle = 0;
    //     chosen_distance = 0;
    //     run = 0;

    //     // Reset buffer
    //     nav.reset_buffer(lidar_buffer, MAX_SERVO_ANGLE + 1);

    // }

    return 0;
}


