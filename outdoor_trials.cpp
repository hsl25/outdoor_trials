#include <stdio.h>
#include "pico/stdlib.h"

// File includes
#include "buffer.hpp"
#include "driving.hpp"
#include "tof.hpp"

// Instantiate objects
TOF tof;
Drive drive;
Buffer buffer;

int main() {
    stdio_init_all();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}


