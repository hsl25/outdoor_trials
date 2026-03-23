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
    // Initialise serial monitor just in case
    stdio_init_all();

    // Initialise I2C and UART
    tof.init_i2c();
    tof.init_uart();

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


