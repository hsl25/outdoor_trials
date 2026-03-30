#ifndef SERVO_HPP
#define SERVO_HPP

#include <pico/stdlib.h>
#include <hardware/pwm.h>

// PWM defines
#define PWM_PIN 18 // GP18 used for PWM control of the front servo motor
#define PWM_DIVIDER 64.0f // The servo needs to receive a 50Hz signal, so we need to adjust the divider and TOP value. Remember, the clk frequency on the RP2040 is around 125MHz
#define PWM_TOP 39062 // Will adjust this later but I just copied this from driving.hpp for now
#define SERVO_DUTY_CYCLE 0.5 // Will be tuned later
#define PWM_PERIOD 20.0 // 20ms period for standard servo control

// Servo defines
#define SWEEP_DELAY 10 // 10ms delay between angle incremements when sweeping the servo back and forth
#define CALIBRATION_SWEEPS 10 // Before the rover starts moving, the servo will scan back and forth 10 times for distance cailbration
#define ZERO_PULSE 1.0 // PWM pulse time for 0 degrees
#define NINETY_PULSE 1.5 // PWM pulse time for 90 degrees
#define ONE_EIGHTY_PULSE 2.0 // PWM pulse time for 180 degrees
#define CENTRAL_ANGLE 90.0

class Servo {
    public:
        Servo();
        void init_pwm(uint32_t pin, float duty);
        void init_servo();
        void set_angle(int angle); 
        // Not needed for the time being
        // void set_zero();
        // void set_ninety();
        // void set_one_eighty();
        void single_sweep();
    private:
        int default_angle; // This is the angle for a single position of the servo 
        int sweep_angle; // This is the angle that will be adjusted whent he servo is sweeping back and forth
};

#endif