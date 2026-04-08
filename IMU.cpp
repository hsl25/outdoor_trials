#include "IMU.hpp"

#include <cmath>
#include <cstdio>

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kRadToDeg = 57.29577951308232f;
}

IMU::IMU(i2c_inst_t* i2c_port,
         uint sda_pin,
         uint scl_pin,
         uint8_t address)
    : i2c_port_(i2c_port),
      sda_pin_(sda_pin),
      scl_pin_(scl_pin),
      address_(address),
      mpu_(mpu6050_init(i2c_port, address)),
      data_{},
      initialized_(false),
      first_update_(true),
      last_update_time_(nil_time),
      roll_deg_(0.0f),
      pitch_deg_(0.0f),
      yaw_deg_(0.0f),
      gz_bias_dps_(0.0f),
      gz_lp_dps_(0.0f),
      complementary_alpha_(0.98f),
      yaw_deadband_dps_(0.4f),
      acc_stationary_threshold_mps2_(0.25f),
      gyro_stationary_threshold_dps_(1.5f),
      stationary_(false) {
    data_.valid = false;
    data_.stationary = false;
}

bool IMU::init() {
    // I2C init: GP20 = SDA, GP21 = SCL on i2c0
    i2c_init(i2c_port_, 400 * 1000);

    gpio_set_function(sda_pin_, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin_, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin_);
    gpio_pull_up(scl_pin_);

    mpu_ = mpu6050_init(i2c_port_, address_);

    if (!mpu6050_begin(&mpu_)) {
        initialized_ = false;
        data_.valid = false;
        return false;
    }

    // Enable measurements
    mpu6050_set_accelerometer_measuring(&mpu_, 1);
    mpu6050_set_gyroscope_measuring(&mpu_, 1);
    mpu6050_set_temperature_measuring(&mpu_, 1);

    // Optional: a little filtering inside MPU6050
    mpu6050_set_dlpf_mode(&mpu_, MPU6050_DLPF_3);

    // Gyro calibration from the library itself
    // Should be done while sensor is stationary during startup
    mpu6050_calibrate_gyro(&mpu_, 100);
    mpu6050_set_threshold(&mpu_, 1);

    initialized_ = true;
    first_update_ = true;
    data_.valid = false;
    return true;
}

bool IMU::computeDt(float& dt_s) {
    absolute_time_t now = get_absolute_time();

    if (first_update_) {
        last_update_time_ = now;
        first_update_ = false;
        dt_s = 0.01f;
        return true;
    }

    int64_t dt_us = absolute_time_diff_us(last_update_time_, now);
    last_update_time_ = now;

    dt_s = static_cast<float>(dt_us) * 1e-6f;

    // Reject unreasonable dt jumps
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = 0.01f;
    }

    return true;
}

void IMU::updateStationaryState(float ax, float ay, float az,
                                float gx, float gy, float gz) {
    const float acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    const float gyro_norm = std::sqrt(gx * gx + gy * gy + gz * gz);

    const bool acc_still = std::fabs(acc_norm - kGravity) < acc_stationary_threshold_mps2_;
    const bool gyro_still = gyro_norm < gyro_stationary_threshold_dps_;

    stationary_ = acc_still && gyro_still;
}

void IMU::wrapYaw() {
    while (yaw_deg_ > 180.0f) yaw_deg_ -= 360.0f;
    while (yaw_deg_ < -180.0f) yaw_deg_ += 360.0f;
}

bool IMU::update() {
    if (!initialized_) {
        data_.valid = false;
        return false;
    }

    float dt_s = 0.01f;
    computeDt(dt_s);

    if (!mpu6050_event(&mpu_)) {
        data_.valid = false;
        return false;
    }

    mpu6050_vectorf* acc = mpu6050_get_accelerometer(&mpu_);
    mpu6050_vectorf* gyro = mpu6050_get_gyroscope(&mpu_);
    const float temp_c = mpu6050_get_temperature_c(&mpu_);

    if (!acc || !gyro) {
        data_.valid = false;
        return false;
    }

    // Sensor outputs from library:
    // acc in m/s^2, gyro in deg/s
    const float ax = acc->x;
    const float ay = acc->y;
    const float az = acc->z;

    const float gx = gyro->x;
    const float gy = gyro->y;
    const float gz = gyro->z;

    // 1) Roll and pitch from accelerometer
    const float roll_acc_deg =
        std::atan2(ay, az) * kRadToDeg;

    const float pitch_acc_deg =
        std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;

    // 2) Complementary filter for roll / pitch
    roll_deg_ =
        complementary_alpha_ * (roll_deg_ + gx * dt_s) +
        (1.0f - complementary_alpha_) * roll_acc_deg;

    pitch_deg_ =
        complementary_alpha_ * (pitch_deg_ + gy * dt_s) +
        (1.0f - complementary_alpha_) * pitch_acc_deg;

    // 3) Stationary detection
    updateStationaryState(ax, ay, az, gx, gy, gz);

    // 4) Low-pass filter on gz
    // keep most dynamics while suppressing spikes
    const float gz_lp_alpha = 0.85f;
    gz_lp_dps_ = gz_lp_alpha * gz_lp_dps_ + (1.0f - gz_lp_alpha) * gz;

    // 5) Slowly adapt yaw gyro bias when stationary
    if (stationary_) {
        const float bias_alpha = 0.995f;
        gz_bias_dps_ = bias_alpha * gz_bias_dps_ + (1.0f - bias_alpha) * gz_lp_dps_;
    }

    // 6) Yaw integration with bias correction
    float gz_corrected_dps = gz_lp_dps_ - gz_bias_dps_;

    // Deadband to suppress tiny drift
    if (std::fabs(gz_corrected_dps) < yaw_deadband_dps_) {
        gz_corrected_dps = 0.0f;
    }

    // Integrate yaw
    yaw_deg_ += gz_corrected_dps * dt_s;
    wrapYaw();

    // 7) Fill output
    data_.ax_mps2 = ax;
    data_.ay_mps2 = ay;
    data_.az_mps2 = az;

    data_.gx_dps = gx;
    data_.gy_dps = gy;
    data_.gz_dps = gz;

    data_.temperature_c = temp_c;

    data_.roll_deg = roll_deg_;
    data_.pitch_deg = pitch_deg_;
    data_.yaw_deg = yaw_deg_;

    data_.stationary = stationary_;
    data_.valid = true;

    return true;
}

ImuData IMU::read() const {
    return data_;
}

void IMU::resetYaw() {
    yaw_deg_ = 0.0f;
    data_.yaw_deg = 0.0f;
}

void IMU::setComplementaryAlpha(float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    complementary_alpha_ = alpha;
}

void IMU::setYawDeadbandDps(float deadband_dps) {
    if (deadband_dps < 0.0f) deadband_dps = 0.0f;
    yaw_deadband_dps_ = deadband_dps;
}