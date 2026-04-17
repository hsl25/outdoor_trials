#include "common.hpp"
#include "pico/stdlib.h"
#include <stdint.h>
#include <cstdint>

void state_manager_init();

void state_manager_update_tof(uint16_t front_mm,
                              uint16_t rear_mm,
                              bool valid,
                              uint32_t ts_ms);

void state_manager_update_imu(float roll_deg,
                              float pitch_deg,
                              float yaw_deg,
                              float yaw_rate_dps,
                              bool valid,
                              uint32_t ts_ms);

void state_manager_update_motor(const uint8_t pwm_percent[6],
                                bool valid,
                                uint32_t ts_ms);

void state_manager_set_mode(Rovermode mode);

void state_manager_set_gui_connected(bool connected);

RoverState state_manager_get_snapshot();