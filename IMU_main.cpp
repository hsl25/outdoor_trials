#include <stdio.h>
#include "pico/stdlib.h"
#include "IMU.hpp"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    IMU imu(i2c0, 20, 21, MPU6050_ADDRESS_A0_GND);

    if (!imu.init()) {
        while (true) {
            printf("IMU init failed\r\n");
            sleep_ms(1000);
        }
    }

    while (true) {
        if (imu.update()) {
            ImuData data = imu.read();

            if (data.valid) {
                printf("roll=%.2f, pitch=%.2f, yaw=%.2f\r\n",
                       data.roll_deg,
                       data.pitch_deg,
                       data.yaw_deg);
            } else {
                printf("IMU data invalid\r\n");
            }
        } else {
            printf("IMU update failed\r\n");
        }

        sleep_ms(20);  // 50Hz
    }

    return 0;
}