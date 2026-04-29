#ifndef TOF_HPP
#define TOF_HPP

#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "vl53l0x_api.h"

// Front LiDAR I2C defines 
#define I2C_FRONT_PORT i2c1
#define FRONT_SDA_PIN 14
#define FRONT_SCL_PIN 15

// Rear LiDAR I2C defines 
#define I2C_REAR_PORT i2c0
#define REAR_SDA_PIN 16
#define REAR_SCL_PIN 17

#define I2C_BAUDRATE 100000

// TX/RX defines
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_ID uart0
#define UART_BAUD_RATE 115200 

#define MAX_RANGE 1200 // Max distance in mm

// Extend the ST dev struct to carry the Pico I2C instance
// This sits alongside I2cDevAddr in the same struct
// Extend the ST dev struct via inheritance so I2cDevAddr is a direct member
struct TOF_Dev_t : public VL53L0X_Dev_t {
    i2c_inst_t *i2c_port;
};

class TOF {
    public:
        TOF();
        void init_front_i2c();
        void init_rear_i2c();
        void init_uart();
        void device_setup();
        void calibration();
        void start_continuous_ranging();
        uint16_t read_tof_continuous();
    private:
        uint32_t ranging_timing_budget = 33000; // Ranging timing budget in microseconds
        TOF_Dev_t front_dev;
        TOF_Dev_t rear_dev;
        VL53L0X_DEV pDev; // points to whichever is active, or keep both
};

#endif

