// Converting from PWM to angle
// Refer to this link: https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/ 
// The PWM of the signal determines the angle the servo motor will rotate to
// A 1.0ms pulse corresponds to 0 degrees
// A 1.5ms pulse corresponds to the central position of 90 degrees
// A 2.0ms pulse corresponds to 180 degrees

#include <iostream>
#include <stdio.h>
#include <cstdint>

// Pico includes
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/pwm.h>

// Header file include
#include "servo.hpp"

// Constructor
Servo::Servo() {
    default_angle = 90; // Default angle for single position
}

void Servo::init_pwm(uint32_t pin, float duty) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    uint ch = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);     // slow down clock
    pwm_config_set_wrap(&config, 20000);        // 20ms period

    pwm_init(slice, &config, true);

    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    pwm_set_chan_level(slice, ch, duty * 20000);
}

void Servo::init_servo() {
    init_pwm(SERVO1_PWM_PIN, SERVO_DUTY_CYCLE);
    init_pwm(SERVO2_PWM_PIN, SERVO_DUTY_CYCLE);
}

// This function will set the angle of the servo to a specific angle instead of sweeping
// The angle set is determined via PWM duty cycle
// The angle range is 0 to 180 degrees
void Servo::set_front_angle(int angle) {
    if (angle < 0) angle = 0;
    else if (angle > 180) angle = 180;

    float pulse_ms = 1.0f + (angle / 180.0f); // range: 1.0 to 2.0
    float duty_cycle = pulse_ms / 20.0f;

    uint slice1 = pwm_gpio_to_slice_num(SERVO1_PWM_PIN);
    uint ch1 = pwm_gpio_to_channel(SERVO1_PWM_PIN);

    // Just update the level, don't re-init
    pwm_set_chan_level(slice1, ch1, (uint16_t)(duty_cycle * 20000));
}

void Servo::set_rear_angle(int angle) {
    if (angle < 0) angle = 0;
    else if (angle > 180) angle = 180;

    float pulse_ms = 1.0f + (angle / 180.0f); // range: 1.0 to 2.0
    float duty_cycle = pulse_ms / 20.0f;

    uint slice2 = pwm_gpio_to_slice_num(SERVO2_PWM_PIN);
    uint ch2 = pwm_gpio_to_channel(SERVO2_PWM_PIN);

    // Just update the level, don't re-init
    pwm_set_chan_level(slice2, ch2, (uint16_t)(duty_cycle * 20000));
}

// This function sweeps the servo from 0 degrees to 180 degrees and then back to 0 degrees
void Servo::single_front_sweep() {
    
    // Set the angle initially to 0 degrees
    set_front_angle(0);

    // Sweep from 0 degrees to 180 degrees
    for (int angle = 0; angle < 180; angle++) {
        set_front_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }

    // Now sweep from 180 degrees back to 0 degrees
    for (int angle = 180; angle > 0; angle--) {
        set_front_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }

}

void Servo::single_rear_sweep() {
    // Set the angle initially to 0 degrees
    set_rear_angle(0);

    // Sweep from 0 degrees to 180 degrees
    for (int angle = 0; angle < 180; angle++) {
        set_rear_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }

    // Now sweep from 180 degrees back to 0 degrees
    for (int angle = 180; angle > 0; angle--) {
        set_rear_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }

}