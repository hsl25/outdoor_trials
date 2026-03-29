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

    pwm_set_wrap(slice, TOP);

    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    pwm_set_chan_level(slice, ch, duty * TOP);

    pwm_set_enabled(slice, true);
}

void Servo::init_servo() {
    init_pwm(PWM_PIN, SERVO_DUTY_CYCLE);
}

// This function will set the angle of the servo to a specific angle instead of sweeping
// The angle set is determined via PWM duty cycle
// The angle range is 0 to 180 degrees
void Servo::set_angle(int angle) {
    Servo servo;
    // Prevents error cases
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }

    float pulse_width = 1.0 + (angle / 180.0) * 1.0; // Linear interpolation between 1.0ms and 2.0ms
    float duty_cycle = pulse_width / PWM_PERIOD; // Convert pulse width to duty cycle (assuming a 20ms period)

    // Set the PWM duty cycle to control the servo angle
    // This should set the angle of the servo to the one we chose based on the duty cycle we calculated
    servo.init_pwm(PWM_PIN, duty_cycle);

}

void set_zero() {
    Servo servo;

    float duty_cycle = ZERO_PULSE/ PWM_PERIOD; // Convert pulse width to duty cycle (assuming a 20ms period)

    // Set the PWM duty cycle to control the servo angle
    // This should set the angle of the servo to the one we chose based on the duty cycle we calculated
    servo.init_pwm(PWM_PIN, duty_cycle);
    
}

void set_ninety() {
    Servo servo;

    float duty_cycle = NINETY_PULSE/ PWM_PERIOD; // Convert pulse width to duty cycle (assuming a 20ms period)

    // Set the PWM duty cycle to control the servo angle
    // This should set the angle of the servo to the one we chose based on the duty cycle we calculated
    servo.init_pwm(PWM_PIN, duty_cycle);
    
}

void set_one_eighty() {
    Servo servo;

    float duty_cycle = ONE_EIGHTY_PULSE/ PWM_PERIOD; // Convert pulse width to duty cycle (assuming a 20ms period)

    // Set the PWM duty cycle to control the servo angle
    // This should set the angle of the servo to the one we chose based on the duty cycle we calculated
    servo.init_pwm(PWM_PIN, duty_cycle);
    
}

// This function sweeps the servo from 0 degrees to 180 degrees and then back to 0 degrees
void Servo::single_sweep() {
    // Instantiate servo object
    Servo servo;
    
    // Set the angle initially to 0 degrees
    servo.set_angle(0);

    // Sweep from 0 degrees to 180 degrees
    for (int angle = 0; angle < 180; angle++) {
        servo.set_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }

    // Now sweep from 180 degrees back to 0 degrees
    for (int angle = 180; angle > 0; angle--) {
        servo.set_angle(angle);
        sleep_ms(SWEEP_DELAY);
    }


}