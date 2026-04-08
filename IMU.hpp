#pragma once

#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

extern "C" {
#include "MPU6050.h"
}

struct ImuData {
    float ax_mps2;
    float ay_mps2;
    float az_mps2;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float temperature_c;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    bool stationary;
    bool valid;
};

class IMU {
public:
    IMU(i2c_inst_t* i2c_port = i2c0,
        uint sda_pin = 20,
        uint scl_pin = 21,
        uint8_t address = MPU6050_ADDRESS_A0_GND);

    bool init();
    bool update();
    ImuData read() const;

    void resetYaw();
    void setComplementaryAlpha(float alpha);
    void setYawDeadbandDps(float deadband_dps);

private:
    bool computeDt(float& dt_s);
    void updateStationaryState(float ax, float ay, float az,
                               float gx, float gy, float gz);
    void wrapYaw();

private:
    i2c_inst_t* i2c_port_;
    uint sda_pin_;
    uint scl_pin_;
    uint8_t address_;

    mpu6050_t mpu_;
    ImuData data_;

    bool initialized_;
    bool first_update_;
    absolute_time_t last_update_time_;

    float roll_deg_;
    float pitch_deg_;
    float yaw_deg_;

    float gz_bias_dps_;

    float gz_lp_dps_;

    float complementary_alpha_;
    float yaw_deadband_dps_;

    float acc_stationary_threshold_mps2_;
    float gyro_stationary_threshold_dps_;

    bool stationary_;
};