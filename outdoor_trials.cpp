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

uint16_t front_lidar_buffer[SERVO_MAX_SWEEP_ANGLE - SERVO_MIN_SWEEP_ANGLE + 1] = {0};
uint16_t rear_lidar_buffer[SERVO_MAX_SWEEP_ANGLE - SERVO_MIN_SWEEP_ANGLE + 1] = {0};
int size = SERVO_MAX_SWEEP_ANGLE - SERVO_MIN_SWEEP_ANGLE + 1;

int main() {
    // Initialise serial monitor just in case I need it for debugging 
    stdio_init_all();

    int temp = 360 / SKID_CHECK_ANGLE;

    servo.init_front_servo();
    servo.init_rear_servo();

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
    bool found = false;
    
    sleep_ms(201);

    // nav.print_buffer(front_lidar_buffer, size);

    while (1) {
        for (run = 0; run < temp; run++) {
            // Scan the surroundings and measure distances
            tof.start_continuous_ranging();
            nav.forward_sweep(CALIBRATION_SWEEPS, front_lidar_buffer, size);

            if (nav.check_openings(front_lidar_buffer, size, THRESHOLD_DIST)) {
                

            } else if (!nav.check_openings(front_lidar_buffer, size, THRESHOLD_DIST)) {
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

                    nav.skid_into_position(start_yaw, imu_angle);
                    drive.brake();

                    // Ok now we are in position so we can break and start navigating
                    found = true;
                    break;

                } else {
                    imu.update();
                    start_yaw = imu.read().yaw_deg;

                    if (nav.space_check(front_lidar_buffer, size)) {
                        int max_centre = nav.check_max_range(front_lidar_buffer, size);
                        imu_angle = max_centre + SERVO_MIN_SWEEP_ANGLE - 90;
                        nav.skid_into_position(start_yaw, (float)imu_angle);
                    } else {
                        // completely stuck — rotate by SKID_CHECK_ANGLE and rescan
                        nav.skid_into_position(start_yaw, (float)SKID_CHECK_ANGLE);
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

                // This part only runs if we find a suitable gap
                if (found) {
                    // OK so we know where to go, now we have to calculate how far to travel and also verify with the IMU
                    // The chosen distance is in mm, we need to convert this to metres
                    // This gives time in seconds 
                    float target_m = (chosen_distance / 1000.0f) / (float)DISTANCE_DIVIDER;
                    float drive_time = drive.calc_drive_time(WHEEL_DIAMETER, OLD_MOTOR_RPM, target_m);

                    // Now drive
                    drive.drive_forward();
                    sleep_ms((uint32_t)(drive_time * 1000));
                    drive.brake();

                    // Ok, now we have finished navigating. 
                    // Now we simply scan again and navigate again, and this cycle continues till the rover is turned off via the GUI
                    // We still need to reset everything though

                    // Reset variables
                    num_sweeps = 0;
                    centre_angle = 0;
                    chosen_gap = 0;
                    chosen_distance = 0;
                    run = 0;

                    // Reset buffer
                    nav.reset_buffer(front_lidar_buffer, size);

                    // Reset flag
                    found = false;
                }
            }

        }

    }

    return 0;

}