#include "state_manager.hpp"
#include "pico/time.h"
#include <cstring>   


static RoverState g_state;


void state_manager_init() {
    
    std::memset(&g_state, 0, sizeof(g_state));

    
    g_state.mode = Rovermode::AUTO;

    
    g_state.gui_connected = false;

    
    g_state.tof.valid = false;
    g_state.imu.valid = false;
    g_state.motor.valid = false;

    
    g_state.ts_ms = to_ms_since_boot(get_absolute_time());
}


void state_manager_update_tof(uint16_t front_mm,
                              uint16_t rear_mm,
                              bool valid,
                              uint32_t ts_ms) {

    g_state.tof.front_mm = front_mm;
    g_state.tof.rear_mm  = rear_mm;
    g_state.tof.valid    = valid;
    g_state.tof.ts_ms    = ts_ms;
}


void state_manager_update_imu(float roll_deg,
                              float pitch_deg,
                              float yaw_deg,
                              float yaw_rate_dps,
                              bool valid,
                              uint32_t ts_ms) {

    g_state.imu.roll_deg      = roll_deg;
    g_state.imu.pitch_deg     = pitch_deg;
    g_state.imu.yaw_deg       = yaw_deg;
    g_state.imu.yaw_rate_dps  = yaw_rate_dps;
    g_state.imu.valid         = valid;
    g_state.imu.ts_ms         = ts_ms;
}


void state_manager_update_motor(const uint8_t pwm_percent[6],
                                bool valid,
                                uint32_t ts_ms) {

   
    std::memcpy(g_state.motor.pwm_percent, pwm_percent, 6);

    g_state.motor.valid = valid;
    g_state.motor.ts_ms = ts_ms;
}


void state_manager_set_mode(Rovermode mode) {
    g_state.mode = mode;
}


void state_manager_set_gui_connected(bool connected) {
    g_state.gui_connected = connected;
}


RoverState state_manager_get_snapshot() {
    
    g_state.ts_ms = to_ms_since_boot(get_absolute_time());

    
    return g_state;
}