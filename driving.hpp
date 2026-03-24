#ifndef DRIVING_HPP
#define DRIVING_HPP

#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <cstddef>
#include <iostream>

// ==================== PWM and steering defines ====================
#define PWM_PHASE_CORRECT 1
#define CLOCKS_PER_SEC 1000000
#define STRAIGHT_LINE_TIME 5000
#define TURN_TIME 5000
#define SKID_STEERING_TIME 20000
#define TOP 3124
#define DIVIDER 1
#define NORMAL_DUTY_CYCLE 0.5
#define OUTER_DUTY_CYCLE 0.5 // Refers to the PWM duty cycle of the outer wheels when steering
#define TURN_RADIUS 0.25 // 0.50 metre turning radius
#define TRACK_WIDTH 0.20 // 0.2m distance between the left and right wheels on the same axle

//  ==================== PWM Pins used (1 speed pin per motor) ====================
#define MOTOR1_PWM_PIN 3    // GP3 - motor 1
#define MOTOR2_PWM_PIN 11    // GP11 - motor 2
#define MOTOR3_PWM_PIN 13    // GP3 - motor 3
#define MOTOR4_PWM_PIN 5    // GP5 - motor 4
#define MOTOR5_PWM_PIN 7   // GP7 - motor 5
#define MOTOR6_PWM_PIN 9   // GP9 - motor 6

//  ==================== Direction Pins used (1 direction pin per motor) ====================
#define MOTOR1_DIR_PIN 2  // GP2 - motor 1
#define MOTOR2_DIR_PIN 10  // GP10 - motor 2
#define MOTOR3_DIR_PIN 12  // GP12 - motor 3
#define MOTOR4_DIR_PIN 4  // GP4 - motor 4
#define MOTOR5_DIR_PIN 6 // GP6 - motor 5 
#define MOTOR6_DIR_PIN 8 // GP8 - motor 6 

//  ==================== Motor slices used (1 slice per 2 motors) ====================
#define MOTOR12_SLICE 1 // 1A, 1B
#define MOTOR34_SLICE 2 // 2A, 2B
#define MOTOR56_SLICE 3 // 3A, 3B 

// ==================== Other defines ====================
#define PI 3.141592
#define OLD_MOTOR_RPM 56 // This is the RPM of the old motors at 50% duty cycle 
#define OLD_WHEEL_DIAMETER 0.107 // 107mm diameter wheels on the old motors
#define NEW_WHEEL_DIAMETER 0.140 // 140mm diameter wheels on the new motors

class Drive {
    private:
        const int old_motor_level = TOP * NORMAL_DUTY_CYCLE;
        int new_motor_level;
        float new_motor_pwm;
    public:
        Drive();
        void init_pwm_mode();
        void init_clk_divider();
        void init_pwm(uint32_t pin, float duty);
        void setup_motors();
        void set_motor_output(unsigned int speed_pin, unsigned int dir_pin, float motor_speed);
        void drive_forward();
        void reverse();
        float calc_speed_ratio(float turn_radius, float track_width);
        float calc_pwm(float old_wheel_diameter, float new_wheel_diameter);
        void turn_left(float speed_ratio);
        void turn_right(float speed_ratio);
        void skid_left();
        void skid_right();
        void brake();
    };

#endif 