#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

// File includes
#include "buffer.hpp"
#include "driving.hpp"
#include "tof.hpp"
#include "servo.hpp"

// Defines
#define ROVER_WIDTH 600 // Width of the rover in mm
#define ROVER_LENGTH 440 // Length of rover in mm
#define SAFETY_MARGIN 10 // 10mm safety margin added to the width and length of the rover

// Instantiate objects
TOF tof;
Drive drive;
Buffer buffer;
Servo servo;

uint16_t lidar_buffer[MAX_SERVO_ANGLE + 1];

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

    // Implementing a new algorithm to look for widest gaps instead of looking for largest distances 
    

    return 0;
}


