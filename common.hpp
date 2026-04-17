#pragma once
#include <pico.h>
#include <cstdint>

struct TofState
{
   uint16_t front_mm;
   uint16_t rear_mm;
   bool valid;
   uint32_t ts_ms;
};

struct ImuState
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_rate_dps;
    bool valid;
    uint32_t ts_ms;
};

struct MotorState
{
    uint8_t pwm_percent[6];
    bool valid;
    uint32_t ts_ms;
};

enum class Rovermode{

    AUTO = 0,
    MANUAL = 1
};

struct RoverState
{
    TofState tof;
    ImuState imu;
    MotorState motor;
    Rovermode mode;
    bool gui_connected;
    uint32_t ts_ms;
};