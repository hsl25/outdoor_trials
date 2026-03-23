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
    stdio_init_all();

    // Initialise I2C and UART
    tof.init_i2c();
    tof.init_uart();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}


