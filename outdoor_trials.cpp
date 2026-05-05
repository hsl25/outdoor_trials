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
Servo servo;
IMU imu(i2c0, IMU_SDA_PIN, IMU_SCL_PIN, MPU6050_ADDRESS_A0_GND);
Navigation nav(tof, servo, imu, drive);

const int size = ((SERVO_MAX_SWEEP_ANGLE - SERVO_MIN_SWEEP_ANGLE) / ANGLE_STEP) + 1;
uint16_t front_lidar_buffer[size] = {0};
uint16_t rear_lidar_buffer[size] = {0};

int main() {
    // Initialise serial monitor just in case I need it for debugging 
    stdio_init_all();
    int skid_check_angle = 90;

    int num_rotations = 360 / skid_check_angle;

    servo.init_front_servo();

    // while (!stdio_usb_connected()) {    
    //     sleep_ms(10);
    //     sleep_ms(20);
    // }

    servo.set_front_angle(90);
    sleep_ms(1000);
    printf("scan complete 1\n");

    servo.set_front_angle(SERVO_MIN_SWEEP_ANGLE);
    sleep_ms(1000);
    printf("scan complete 2\n");

    servo.set_front_angle(SERVO_MAX_SWEEP_ANGLE);
    sleep_ms(1000);
    printf("scan complete 3\n");

    // Initialise I2C and UART
    tof.init_front_i2c();

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

    // while (1) {
    //     servo.single_front_sweep();
    //     servo.single_rear_sweep();
    // }

    // Keep track of the number of sweeps the servo has done
    int num_sweeps = 0;;

    // Establish variables
    float chosen_gap = 0.0f;
    float chosen_distance = 0.0f;
    int centre_angle = 0;
    int imu_angle = 0;
    int run = 0;
    float start_yaw = 0;
    int max_index = 0;
    bool nav_success = false;

    sleep_ms(201);

    // while(1) {
    //     nav.skid_into_position(90);
    //     sleep_ms(3000);
    // }

    // nav.print_buffer(front_lidar_buffer, size);

    while (1) {
        for (run = 0; run < num_rotations; run++) {
            // Scan the surroundings and measure distances
            tof.stop_ranging();
            tof.start_continuous_ranging();
            nav.forward_sweep(CALIBRATION_SWEEPS, front_lidar_buffer, size);
            tof.stop_ranging();

            imu.update();
            start_yaw = imu.read().yaw_deg;

            if (nav.check_openings(front_lidar_buffer, size, THRESHOLD_DIST)) {
                max_index = nav.max_index(front_lidar_buffer, size);
                imu_angle = max_index + SERVO_MIN_SWEEP_ANGLE - 90;

                float target_d = (front_lidar_buffer[max_index] / 1000.0f) * THRESHOLD_RATIO;
                float drive_t = drive.calc_drive_time(WHEEL_DIAMETER, OLD_MOTOR_RPM, target_d);

                nav.skid_into_position(imu_angle);
                drive.brake();

                sleep_ms(3000);

                drive.drive_forward();
                sleep_ms((uint32_t)(drive_t * 1000.0f));
                drive.brake();

                nav.reset_buffer(front_lidar_buffer, size);
                skid_check_angle = 90;
                num_rotations = 360 / skid_check_angle;
                nav_success = true;

                break;

            } else {
                // This detect_edge function returns a vector of (hopefully) 2 indexes for the lidar buffer, where edges may exist
                std::vector<int> edges = nav.detect_edge(front_lidar_buffer, size);

                if (edges.size() >= 2) {
                    centre_angle = (int) ((edges[0] + edges[1]) / 2);
                    imu_angle = centre_angle + SERVO_MIN_SWEEP_ANGLE - 90;
                    chosen_distance = (float)front_lidar_buffer[centre_angle];
                    chosen_gap = nav.calc_width(front_lidar_buffer[edges[0]], edges[0], front_lidar_buffer[edges[1]], edges[1]);
                } else {
                    chosen_distance = 0;
                    chosen_gap = 0;
                } 

                if (chosen_gap > (float)ROVER_WIDTH + SAFETY_MARGIN) {
                    // Implementing a function to enable the rover to skid-steer and face a specific angle 
                    
                    imu.update();
                    start_yaw = imu.read().yaw_deg;

                    // A positive yaw indicates a clockwise turn 
                    // A negaive yaw indicates an anticlockwise turn
                    // The current angle interpretation is 0 degrees at the left-most angle, 0 degrees at the normal, and 180 degress at the right-most angle
                    // The IMU interpretation is -90 degrees at the left-most angle, 0 degrees at the normal, and +90 degrees at the right-most angle
                    // Therefore, this is the mapping of servo angle --> IMU yaw angle
                    // 0 degrees --> -90 degrees
                    // 90 degrees --> 0 degrees
                    // 180 degrees --> +90 degrees
                    // Therefore, I need to subtract 90 degrees form 'chosen_angle'

                    nav.skid_into_position(imu_angle);
                    drive.brake();

                    sleep_ms(3000);

                    // The chosen distance is in mm, we need to convert this to metres
                    // This gives time in seconds 
                    float target_m = (chosen_distance / 1000.0f) / (float)DISTANCE_DIVIDER;
                    float drive_time = drive.calc_drive_time(WHEEL_DIAMETER, OLD_MOTOR_RPM, target_m);

                    // Now drive
                    drive.drive_forward();
                    sleep_ms((uint32_t)(drive_time * 1000));
                    drive.brake();
                    tof.stop_ranging();

                    // Reset variables
                    num_sweeps = 0;
                    centre_angle = 0;
                    chosen_gap = 0;
                    chosen_distance = 0;

                    // Reset buffer
                    nav.reset_buffer(front_lidar_buffer, size);

                    // Now break out of the loop
                    break;

                } else {
                    imu.update();
                    start_yaw = imu.read().yaw_deg;

                    if (nav.space_check(front_lidar_buffer, size)) {
                        int max_centre = nav.check_max_range(front_lidar_buffer, size, MAX_RANGE);
                        imu_angle = max_centre + SERVO_MIN_SWEEP_ANGLE - 90;
                        nav.skid_into_position((float)imu_angle);
                        drive.brake();
                    } else {
                        // completely stuck — rotate by SKID_CHECK_ANGLE and rescan
                        nav.skid_into_position((float)skid_check_angle);
                        drive.brake();
                    }

                    drive.brake();

                    // Reset variables
                    num_sweeps = 0;
                    centre_angle = 0;
                    chosen_gap = 0;
                    chosen_distance = 0;
                    imu_angle = 0;

                    // Reset buffer
                    nav.reset_buffer(front_lidar_buffer, size);
                    
                }

            }

        }

        if (run == num_rotations - 1) {
            // If we reach the end, modify the checking angle and try again
            skid_check_angle = std::max(skid_check_angle - 30, 15); 
            num_rotations = 360 / skid_check_angle;
            run = 0; 
        }

    }

    return 0;

}

