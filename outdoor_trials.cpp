#include <stdio.h>
#include "pico/stdlib.h"

// File includes
#include "buffer.hpp"
#include "driving.hpp"
#include "tof.hpp"
#include "servo.hpp"

// Instantiate objects
TOF tof;
Drive drive;
Buffer buffer;
Servo servo;

int main() {
    // Initialise serial monitor just in case I need it for debugging 
    stdio_init_all();

    // Initialise I2C and UART
    tof.init_i2c();
    tof.init_uart();

    // Setting up the VL53L0X LiDAR and checking for errors
    tof.device_setup();

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

    // Set up a flag to indicate when the servo has reached the end of the sweep
    bool servo_end_flag = 0;

    // Keep track of the number of sweeps the servo has done
    int num_sweeps = 0;

    // 1. Set the angle of the servo to 0 degrees - this is done in the for loop when i = 0
    while (num_sweeps < CALIBRATION_SWEEPS) {
        while (!servo_end_flag) {
            for (int i = 0; i < MAX_SERVO_ANGLE; i++) {
                // Now increment the angle of the servo by 1 degree
                servo.set_angle(i);

                // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
                // Then, the data point is added to the buffer
                // This is step 3
                uint16_t lidar_data = tof.read_tof_continuous();
                // We do 2 * CALIBRATION_SWEEPS because each sweep passes through an angle twice
                // I define 1 sweep as 0 -> 180 and 180 -> 0
                float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS); 
                buffer.add_calib_sample(temp, i); 

                // Toggle the flag if the servo reaches the end of the sweep
                if (i == MAX_SERVO_ANGLE) {
                    servo_end_flag = !servo_end_flag; // servo_end_flag is equal to 1
                }

            }  
        }

        while (servo_end_flag) {
            for (int i = MAX_SERVO_ANGLE; i > 0; i--) {
                // Now increment the angle of the servo by 1 degree
                servo.set_angle(i);

                // When tof.read_continuous() runs, it checks whether data is available, and if so, it returns 1 data point
                // Then, the data point is added to the buffer
                // This is step 3
                uint16_t lidar_data = tof.read_tof_continuous();
                float temp = ((float) lidar_data) / (2 * CALIBRATION_SWEEPS);
                buffer.add_calib_sample(temp, i);  

                // Toggle the flag if the servo reaches the end of the sweep
                if (i == 0) {
                    servo_end_flag = !servo_end_flag;
                }
            } 
        }
        
        num_sweeps++;

    }
    

    // Before the rover starts moving, scan the surroundings N times to get mean values for distances 
    // This is the initial calibration stage
    // Still need to modify this function to add data into a buffer and calculate mean distances
    for (unsigned int i = 0; i < CALIBRATION_SWEEPS; i++) {
        servo.single_sweep();
    }

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}


