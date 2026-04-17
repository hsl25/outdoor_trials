#include "json_builder.hpp"
#include <string>
#include <cstdint>

static const char* rover_mode_to_string(Rovermode mode) {
    switch (mode) {
        case Rovermode::AUTO:   return "AUTO";
        case Rovermode::MANUAL: return "MANUAL";
        default:                return "UNKNOWN";
    }
}

static std::string bool_to_json(bool value) {
    return value ? "true" : "false";
}

std::string json_build_state(const RoverState& state) {
    std::string json = "{";

    json += "\"tof\":{";
    json += "\"front\":" + std::to_string(state.tof.front_mm) + ",";
    json += "\"rear\":" + std::to_string(state.tof.rear_mm) + ",";
    json += "\"valid\":" + bool_to_json(state.tof.valid) + ",";
    json += "\"ts_ms\":" + std::to_string(state.tof.ts_ms);
    json += "},";

  
    json += "\"imu\":{";
    json += "\"roll\":" + std::to_string(state.imu.roll_deg) + ",";
    json += "\"pitch\":" + std::to_string(state.imu.pitch_deg) + ",";
    json += "\"yaw\":" + std::to_string(state.imu.yaw_deg) + ",";
    json += "\"yaw_rate\":" + std::to_string(state.imu.yaw_rate_dps) + ",";
    json += "\"valid\":" + bool_to_json(state.imu.valid) + ",";
    json += "\"ts_ms\":" + std::to_string(state.imu.ts_ms);
    json += "},";

    
    json += "\"motor\":{";
    json += "\"pwm\":[";
    for (int i = 0; i < 6; ++i) {
        json += std::to_string(state.motor.pwm_percent[i]);
        if (i < 5) {
            json += ",";
        }
    }
    json += "],";
    json += "\"valid\":" + bool_to_json(state.motor.valid) + ",";
    json += "\"ts_ms\":" + std::to_string(state.motor.ts_ms);
    json += "},";

    
    json += "\"mode\":\"" + std::string(rover_mode_to_string(state.mode)) + "\",";
    json += "\"gui_connected\":" + bool_to_json(state.gui_connected) + ",";
    json += "\"ts_ms\":" + std::to_string(state.ts_ms);

    json += "}";

    return json;
}