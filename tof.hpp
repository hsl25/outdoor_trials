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

#define I2C_BAUDRATE 100000

#define MAX_RANGE 1400 // Max distance in mm

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
        void device_setup();
        void calibration();
        void start_continuous_ranging();
        uint16_t read_tof_continuous();
        void stop_ranging();
    private:
        uint32_t ranging_timing_budget = 33000; // Ranging timing budget in microseconds
        TOF_Dev_t front_dev;
        TOF_Dev_t rear_dev;
        VL53L0X_DEV pDev; // points to whichever is active, or keep both
};

#endif

