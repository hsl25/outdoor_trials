// Motor diagram:
//   --------- FRONT ---------
//  |                        |
//  |                        |
//  O== Motor 1    Motor 4 ==O 
//  |                        |
//  |                        |
//  O== Motor 2    Motor 5 ==O 
//  |                        |  
//  |                        |
//  O== Motor 3    Motor 6 ==O
//  |                        |
//  |                        |
//   ---------- BACK ---------
//
//   <--- LEFT     RIGHT --->
// 
// Motors 1 and 4 will be the new motors
// The rest will be old motors

#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <cstddef>

// Pico includes
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/pwm.h>

#include "driving.hpp"

Drive::Drive() {
    new_motor_level = TOP * calc_pwm(OLD_WHEEL_DIAMETER, NEW_WHEEL_DIAMETER);
    new_motor_pwm = calc_pwm(OLD_WHEEL_DIAMETER, NEW_WHEEL_DIAMETER);
}

// ---------------- PWM CONFIGURATION ----------------
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

void Drive::init_pwm(uint32_t pin, float duty) {

    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    uint ch = pwm_gpio_to_channel(pin);

    pwm_set_wrap(slice, TOP);

    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    pwm_set_chan_level(slice, ch, duty * TOP);

    pwm_set_enabled(slice, true);
}

float Drive::calc_pwm(float old_wheel_diameter, float new_wheel_diameter) {
    // Calculate the linear velocity of the old motors at 50% duty cycle in m/
    float V = PI * old_wheel_diameter * (OLD_MOTOR_RPM / 60.0f);

    // Now work backwards and figure out the PWM needed for the new motors
    // This calculates the RPM of the new motors at the same linear velocity as the old motors, given the new wheel diameter
    float new_motor_rpm = V / (PI * new_wheel_diameter) * 60.0f;

    // Return the value of the PWM duty cycle needed for the new motors 
    return (new_motor_rpm / OLD_MOTOR_RPM) * NORMAL_DUTY_CYCLE;
}

// ---------------- MOTOR SETUP ----------------

void Drive::setup_motors() {

    // Initialise PWM outputs
    init_pwm(MOTOR1_PWM_PIN, calc_pwm(OLD_WHEEL_DIAMETER, NEW_WHEEL_DIAMETER));
    init_pwm(MOTOR2_PWM_PIN, NORMAL_DUTY_CYCLE);
    init_pwm(MOTOR3_PWM_PIN, NORMAL_DUTY_CYCLE);
    init_pwm(MOTOR4_PWM_PIN, calc_pwm(OLD_WHEEL_DIAMETER, NEW_WHEEL_DIAMETER));
    init_pwm(MOTOR5_PWM_PIN, NORMAL_DUTY_CYCLE);
    init_pwm(MOTOR6_PWM_PIN, NORMAL_DUTY_CYCLE);

    // Direction pins
    gpio_init(MOTOR1_DIR_PIN);
    gpio_set_dir(MOTOR1_DIR_PIN, GPIO_OUT);

    gpio_init(MOTOR2_DIR_PIN);
    gpio_set_dir(MOTOR2_DIR_PIN, GPIO_OUT);

    gpio_init(MOTOR3_DIR_PIN);
    gpio_set_dir(MOTOR3_DIR_PIN, GPIO_OUT);

    gpio_init(MOTOR4_DIR_PIN);
    gpio_set_dir(MOTOR4_DIR_PIN, GPIO_OUT);

    gpio_init(MOTOR5_DIR_PIN);
    gpio_set_dir(MOTOR5_DIR_PIN, GPIO_OUT);

    gpio_init(MOTOR6_DIR_PIN);
    gpio_set_dir(MOTOR6_DIR_PIN, GPIO_OUT);

    // Set all motors to forward
    gpio_put(MOTOR1_DIR_PIN, 0);
    gpio_put(MOTOR2_DIR_PIN, 0);
    gpio_put(MOTOR3_DIR_PIN, 0);
    gpio_put(MOTOR4_DIR_PIN, 0);
    gpio_put(MOTOR5_DIR_PIN, 0);
    gpio_put(MOTOR6_DIR_PIN, 0);

    // Set initial speed
    pwm_set_gpio_level(MOTOR1_PWM_PIN, new_motor_level);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, old_motor_level);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, old_motor_level);
    pwm_set_gpio_level(MOTOR4_PWM_PIN, new_motor_level);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, old_motor_level);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, old_motor_level);
}

// ---------------- MOTOR CONTROL ----------------
void Drive::set_motor_output(unsigned int speed_pin, unsigned int dir_pin, float motor_speed) {
    if (motor_speed > 1.0f) motor_speed = 1.0f;
    if (motor_speed < 0.0f) motor_speed = 0.0f;

    gpio_put(dir_pin, 0);   // Forward

    pwm_set_gpio_level(speed_pin, motor_speed * TOP);
}

// ---------------- BASIC MOVEMENT ----------------
void Drive::drive_forward() {

    set_motor_output(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, new_motor_pwm);
    set_motor_output(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, NORMAL_DUTY_CYCLE);
    set_motor_output(MOTOR3_PWM_PIN, MOTOR3_DIR_PIN, NORMAL_DUTY_CYCLE);
    set_motor_output(MOTOR4_PWM_PIN, MOTOR4_DIR_PIN, new_motor_pwm);
    set_motor_output(MOTOR5_PWM_PIN, MOTOR5_DIR_PIN, NORMAL_DUTY_CYCLE);
    set_motor_output(MOTOR6_PWM_PIN, MOTOR6_DIR_PIN, NORMAL_DUTY_CYCLE);

}

// ---------------- TURN CALCULATIONS ----------------
float Drive::calc_speed_ratio(float turn_radius, float track_width) {

    float speed_ratio = (turn_radius + (track_width / 2)) / (turn_radius - (track_width / 2));

    return speed_ratio;
}

// ---------------- DIFFERENTIAL TURN ----------------`
void Drive::turn_left(float speed_ratio) {

    float old_motor_inner_pwm = OUTER_DUTY_CYCLE / speed_ratio;
    float new_motor_inner_pwm = new_motor_pwm / speed_ratio;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, new_motor_inner_pwm * TOP);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, old_motor_inner_pwm * TOP);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, old_motor_inner_pwm * TOP);

    pwm_set_gpio_level(MOTOR4_PWM_PIN, new_motor_pwm * TOP);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, OUTER_DUTY_CYCLE * TOP);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, OUTER_DUTY_CYCLE * TOP);
}

void Drive::turn_right(float speed_ratio) {

    float old_motor_inner_pwm = OUTER_DUTY_CYCLE / speed_ratio;
    float new_motor_inner_pwm = new_motor_pwm / speed_ratio;

    pwm_set_gpio_level(MOTOR1_PWM_PIN, new_motor_pwm * TOP);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, OUTER_DUTY_CYCLE * TOP);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, OUTER_DUTY_CYCLE * TOP);

    pwm_set_gpio_level(MOTOR4_PWM_PIN, new_motor_inner_pwm * TOP);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, old_motor_inner_pwm * TOP);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, old_motor_inner_pwm * TOP);
}


// ---------------- SKID STEERING ----------------
void Drive::skid_left() {

    brake();
    sleep_ms(50);

    // 0 forwardsm, 1 backwards
    gpio_put(MOTOR1_DIR_PIN, 1);
    gpio_put(MOTOR2_DIR_PIN, 1);
    gpio_put(MOTOR3_DIR_PIN, 1);
    gpio_put(MOTOR4_DIR_PIN, 0);
    gpio_put(MOTOR5_DIR_PIN, 0);
    gpio_put(MOTOR6_DIR_PIN, 0);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, TOP * new_motor_pwm);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR4_PWM_PIN, TOP * new_motor_pwm);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
}


void Drive::skid_right() {

    brake();
    sleep_ms(50);

    gpio_put(MOTOR1_DIR_PIN, 0);
    gpio_put(MOTOR2_DIR_PIN, 0);
    gpio_put(MOTOR3_DIR_PIN, 0);
    gpio_put(MOTOR4_DIR_PIN, 1);
    gpio_put(MOTOR5_DIR_PIN, 1);
    gpio_put(MOTOR6_DIR_PIN, 1);

    pwm_set_gpio_level(MOTOR1_PWM_PIN, TOP * new_motor_pwm);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR3_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR4_PWM_PIN, TOP * new_motor_pwm);
    pwm_set_gpio_level(MOTOR5_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
    pwm_set_gpio_level(MOTOR6_PWM_PIN, TOP * NORMAL_DUTY_CYCLE);
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