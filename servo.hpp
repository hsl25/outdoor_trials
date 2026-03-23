#ifndef SERVO_HPP
#define SERVO_HPP

// PWM defines
#define PWM_PIN 18 // GP18 used for PWM control of the front servo motor
#define TOP 3214 // Will adjust this later but I just copied this from driving.hpp for now
#define SERVO_DUTY_CYCLE 0.5 // Will be tuned later
#define PWM_PERIOD 20.0; // 20ms period for standard servo control

class Servo {
    public:
        Servo();
        void init_pwm(uint32_t pin, float duty);
        void init_servo();
        void set_angle(float angle);
        void single_sweep();
    private:
        int single_angle; // This is the angle for a single position of the servo 
        int sweep_angle; // This is the angle that will be adjusted whent he servo is sweeping back and forth
        // Pulse times for fixed servo angles
        float zero_pulse = 1.0;
        float ninety_pulse = 1.5;
        float one_eighty_pulse = 2.0;
};

#endif