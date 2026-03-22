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

}

void Servo::init_pwm(uint32_t pin, float duty) {

    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    uint ch = pwm_gpio_to_channel(pin);

    pwm_set_wrap(slice, TOP);

    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    pwm_set_chan_level(slice, ch, duty * TOP);

    pwm_set_enabled(slice, true);
}

void Servo::init_servo() {
    init_pwm(PWM_PIN, SERVO_DUTY_CYCLE);
}

void set_angle(float angle) {
    
}