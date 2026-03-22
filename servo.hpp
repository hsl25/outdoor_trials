#ifndef SERVO_HPP
#define SERVO_HPP

// PWM defines
#define PWM_PIN 18 // GP18 used for PWM control of the front servo motor
#define TOP 3214 // Will adjust this later but I just copied this from driving.hpp for now
#define SERVO_DUTY_CYCLE 0.5 // Will be tuned later

class Servo {
    public:
        Servo();
        void init_pwm(uint32_t pin, float duty);
        void init_servo();
        void set_angle(float angle);
        void single_sweep();
    private:
        float angle;
};

#endif