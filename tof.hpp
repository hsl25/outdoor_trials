#ifndef TOF_HPP
#define TOF_HPP

#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "vl53l0x_api.h"

// I2C defines 
#define I2C_PORT i2c0
#define SDA_PIN 12
#define SCL_PIN 13
#define I2C_BAUDRATE 100000

// TX/RX defines
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_ID uart0
#define UART_BAUD_RATE 115200

class TOF {
    public:
        TOF();
        void init_i2c();
        void init_uart();
        void device_setup();
        void calibration();
        void start_continuous_ranging();
        uint16_t read_tof_continuous();
    private:
        uint32_t ranging_timing_budget = 33000; // Ranging timing budget in microseconds
        VL53L0X_Dev_t dev;
        VL53L0X_DEV pDev;
};

#endif

