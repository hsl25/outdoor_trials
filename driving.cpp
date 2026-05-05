#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <cstddef>
#include <math.h>

#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/pwm.h>

#include "driving.hpp"

Drive::Drive() {
    new_motor_pwm   = calc_pwm(WHEEL_DIAMETER, WHEEL_DIAMETER);
    new_motor_level = (int)(TOP * new_motor_pwm);
}

// ---------------- PWM CONFIGURATION ----------------
// NOTE: init_pwm_mode and init_clk_divider must be called BEFORE setup_motors.
// They configure phase-correct mode and clock divider on every slice.
// init_pwm (called inside setup_motors) then sets wrap and enables the slice —
// it must NOT call pwm_set_phase_correct or pwm_set_clkdiv again because that
// would overwrite the values set here.

void Drive::init_pwm_mode() {
    uint slice1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    uint slice3 = pwm_gpio_to_slice_num(MOTOR3_PWM_PIN);
    uint slice4 = pwm_gpio_to_slice_num(MOTOR4_PWM_PIN);
    uint slice5 = pwm_gpio_to_slice_num(MOTOR5_PWM_PIN);
    uint slice6 = pwm_gpio_to_slice_num(MOTOR6_PWM_PIN);

    pwm_set_phase_correct(slice1, true);
    pwm_set_phase_correct(slice2, true);
    pwm_set_phase_correct(slice3, true);
    pwm_set_phase_correct(slice4, true);
    pwm_set_phase_correct(slice5, true);
    pwm_set_phase_correct(slice6, true);
}

void Drive::init_clk_divider() {
    uint slice1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    uint slice2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    uint slice3 = pwm_gpio_to_slice_num(MOTOR3_PWM_PIN);
    uint slice4 = pwm_gpio_to_slice_num(MOTOR4_PWM_PIN);
    uint slice5 = pwm_gpio_to_slice_num(MOTOR5_PWM_PIN);
    uint slice6 = pwm_gpio_to_slice_num(MOTOR6_PWM_PIN);

    pwm_set_clkdiv(slice1, DIVIDER);
    pwm_set_clkdiv(slice2, DIVIDER);
    pwm_set_clkdiv(slice3, DIVIDER);
    pwm_set_clkdiv(slice4, DIVIDER);
    pwm_set_clkdiv(slice5, DIVIDER);
    pwm_set_clkdiv(slice6, DIVIDER);
}

// ---------------- SINGLE PWM INIT ----------------
// Correct order: assign GPIO function → set wrap → set level to 0 → enable.
// Phase-correct and clkdiv are already set by init_pwm_mode/init_clk_divider.
// The 'duty' parameter is intentionally ignored here — all motors start at 0.
// Speed is applied later by drive_forward(), skid_left(), etc.

void Drive::init_pwm(uint32_t pin, float /*duty*/) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    uint ch    = pwm_gpio_to_channel(pin);

    pwm_set_wrap(slice, TOP);           // must come before set_chan_level
    pwm_set_chan_level(slice, ch, 0);   // start stopped
    pwm_set_enabled(slice, true);       // enable last
}

// ---------------- SPEED HELPERS ----------------
float Drive::calc_pwm(float old_wheel_diameter, float new_wheel_diameter) {
    float V           = PI * old_wheel_diameter * (OLD_MOTOR_RPM / 60.0f);
    float new_motor_rpm = V / (PI * new_wheel_diameter) * 60.0f;
    float duty        = (new_motor_rpm / OLD_MOTOR_RPM) * NORMAL_DUTY_CYCLE;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    return duty;
}

float Drive::calc_drive_time(float diameter, float RPM, float distance) {
    float speed = PI * diameter * (RPM / 60.0f);   // RPM → rev/s → m/s
    if (speed <= 0.0f) return 0.0f;
    return distance / speed;
}

// ---------------- MOTOR SETUP ----------------
void Drive::setup_motors() {
    // Init PWM pins first
    init_pwm(MOTOR1_PWM_PIN, 0.0f);
    init_pwm(MOTOR2_PWM_PIN, 0.0f);
    init_pwm(MOTOR3_PWM_PIN, 0.0f);
    init_pwm(MOTOR4_PWM_PIN, 0.0f);
    init_pwm(MOTOR5_PWM_PIN, 0.0f);
    init_pwm(MOTOR6_PWM_PIN, 0.0f);

    // AFTER init_pwm, forcibly reclaim DIR pins as plain GPIO
    // (init_pwm activates the slice which can pull the paired pin into PWM mode)
    gpio_set_function(MOTOR1_DIR_PIN, GPIO_FUNC_SIO);  // ← force back to GPIO
    gpio_set_function(MOTOR2_DIR_PIN, GPIO_FUNC_SIO);
    gpio_set_function(MOTOR3_DIR_PIN, GPIO_FUNC_SIO);
    gpio_set_function(MOTOR4_DIR_PIN, GPIO_FUNC_SIO);
    gpio_set_function(MOTOR5_DIR_PIN, GPIO_FUNC_SIO);
    gpio_set_function(MOTOR6_DIR_PIN, GPIO_FUNC_SIO);

    // Now set them as outputs as normal
    gpio_init(MOTOR1_DIR_PIN);  gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR1_DIR_PIN, 0);
    gpio_init(MOTOR2_DIR_PIN);  gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR2_DIR_PIN, 0);
    gpio_init(MOTOR3_DIR_PIN);  gpio_set_dir(MOTOR3_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR3_DIR_PIN, 0);
    gpio_init(MOTOR4_DIR_PIN);  gpio_set_dir(MOTOR4_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR4_DIR_PIN, 0);
    gpio_init(MOTOR5_DIR_PIN);  gpio_set_dir(MOTOR5_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR5_DIR_PIN, 0);
    gpio_init(MOTOR6_DIR_PIN);  gpio_set_dir(MOTOR6_DIR_PIN, GPIO_OUT);  gpio_put(MOTOR6_DIR_PIN, 0);

    // Zero all levels
    pwm_set_gpio_level(MOTOR1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR4_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, 0);
}

// ---------------- MOTOR CONTROL ----------------
void Drive::set_motor_output(unsigned int speed_pin, unsigned int dir_pin, float motor_speed) {
    if (motor_speed > 1.0f) motor_speed = 1.0f;
    if (motor_speed < 0.0f) motor_speed = 0.0f;
    gpio_put(dir_pin, 0);
    pwm_set_gpio_level(speed_pin, (uint16_t)(motor_speed * TOP));
}

// ---------------- BASIC MOVEMENT ----------------
void Drive::drive_forward() {
    // Direction 1 = forward in your wiring convention
    gpio_put(MOTOR1_DIR_PIN, 1);
    gpio_put(MOTOR2_DIR_PIN, 1);
    gpio_put(MOTOR3_DIR_PIN, 1);
    gpio_put(MOTOR4_DIR_PIN, 1);
    gpio_put(MOTOR5_DIR_PIN, 1);
    gpio_put(MOTOR6_DIR_PIN, 1);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
}

void Drive::reverse() {
    gpio_put(MOTOR1_DIR_PIN, 0);
    gpio_put(MOTOR2_DIR_PIN, 0);
    gpio_put(MOTOR3_DIR_PIN, 0);
    gpio_put(MOTOR4_DIR_PIN, 0);
    gpio_put(MOTOR5_DIR_PIN, 0);
    gpio_put(MOTOR6_DIR_PIN, 0);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(NORMAL_DUTY_CYCLE * TOP));
}

// ---------------- TURN CALCULATIONS ----------------
float Drive::calc_speed_ratio(float turn_radius, float track_width) {
    return (turn_radius + (track_width / 2.0f)) / (turn_radius - (track_width / 2.0f));
}

// ---------------- DIFFERENTIAL TURN ----------------
void Drive::turn_left(float speed_ratio) {
    float old_motor_inner_pwm = OUTER_DUTY_CYCLE / speed_ratio;
    float new_motor_inner_pwm = new_motor_pwm    / speed_ratio;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(new_motor_inner_pwm * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(old_motor_inner_pwm * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(old_motor_inner_pwm * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(new_motor_pwm       * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(OUTER_DUTY_CYCLE    * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(OUTER_DUTY_CYCLE    * TOP));
}

void Drive::turn_right(float speed_ratio) {
    float old_motor_inner_pwm = OUTER_DUTY_CYCLE / speed_ratio;
    float new_motor_inner_pwm = new_motor_pwm    / speed_ratio;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(new_motor_pwm       * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(OUTER_DUTY_CYCLE    * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(OUTER_DUTY_CYCLE    * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(new_motor_inner_pwm * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(old_motor_inner_pwm * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(old_motor_inner_pwm * TOP));
}

// ---------------- SKID STEERING ----------------
void Drive::skid_left() {
    gpio_put(MOTOR1_DIR_PIN, 1);
    gpio_put(MOTOR2_DIR_PIN, 1);
    gpio_put(MOTOR3_DIR_PIN, 1);
    gpio_put(MOTOR4_DIR_PIN, 0);
    gpio_put(MOTOR5_DIR_PIN, 0);
    gpio_put(MOTOR6_DIR_PIN, 0);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
}

void Drive::skid_right() {
    gpio_put(MOTOR1_DIR_PIN, 0);
    gpio_put(MOTOR2_DIR_PIN, 0);
    gpio_put(MOTOR3_DIR_PIN, 0);
    gpio_put(MOTOR4_DIR_PIN, 1);
    gpio_put(MOTOR5_DIR_PIN, 1);
    gpio_put(MOTOR6_DIR_PIN, 1);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR3_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR4_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * new_motor_pwm    * TOP));
    pwm_set_gpio_level(MOTOR5_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
    pwm_set_gpio_level(MOTOR6_PWM_PIN, (uint16_t)(PWM_SKID_DIVIDER * NORMAL_DUTY_CYCLE * TOP));
}

// ---------------- STOP ----------------
void Drive::brake() {
    pwm_set_gpio_level(MOTOR1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR4_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, 0);
}

// Calculate the time in seconds needed to skid-steer by angle_deg degrees.
// Uses the known wheel RPM, diameter, skid duty cycle, and rover track width.
// The rover rotates about its centre — each wheel travels an arc of radius
// (TRACK_WIDTH / 2). Time = arc_length / wheel_linear_speed.
float Drive::calc_skid_time(float angle_deg) {
    float angle_rad  = fabsf(angle_deg) * (PI / 180.0f);
    float arc_length = angle_rad * (TRACK_WIDTH / 2.0f);

    // Linear speed of wheels at skid duty cycle
    float wheel_speed = PI * WHEEL_DIAMETER * (OLD_MOTOR_RPM / 60.0f) * PWM_SKID_DIVIDER;

    if (wheel_speed <= 0.0f) {
        return 0.0f;
    }
    
    return (arc_length / wheel_speed);
}