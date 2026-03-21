#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include "vl53l0x_api.h"

// I2C defines 
#define I2C_PORT i2c0
#define SDA_PIN 12
#define SCL_PIN 13
#define I2C_BAUDRATE 100000

// TX/RX defines
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_ID uart0
#define UART_BAUD_RATE 115200

int main() {

    stdio_init_all();
    sleep_ms(2000);

    printf("VL53L0X Continuous Ranging Test\n");

    // ---------------- I2C INIT ----------------
    i2c_init(I2C_PORT, I2C_BAUDRATE);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // ---------------- DEVICE SETUP ----------------
    VL53L0X_Dev_t dev;
    VL53L0X_DEV pDev = &dev;

    pDev->I2cDevAddr = 0x29;

    if (VL53L0X_DataInit(pDev) != VL53L0X_ERROR_NONE) {
        printf("DataInit failed\n");
        while (1);
    }

    if (VL53L0X_StaticInit(pDev) != VL53L0X_ERROR_NONE) {
        printf("StaticInit failed\n");
        while (1);
    }

    // ---------------- REQUIRED CALIBRATION ----------------
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if (VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal) != VL53L0X_ERROR_NONE) {
        printf("RefCalibration failed\n");
        while (1);
    }

    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    if (VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads) != VL53L0X_ERROR_NONE) {
        printf("SPAD management failed\n");
        while (1);
    }

    // ---------------- TIMING BUDGET ----------------
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDev, 33000);

     // ---------------- CONTINUOUS MODE ----------------
    VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    if (VL53L0X_StartMeasurement(pDev) != VL53L0X_ERROR_NONE) {
        printf("StartMeasurement failed\n");
        while (1);
    }

    printf("Continuous ranging started...\n");

    VL53L0X_RangingMeasurementData_t data;

    while (1) {

        uint8_t ready = 0;

        // Wait for measurement ready
        while (!ready) {
            VL53L0X_GetMeasurementDataReady(pDev, &ready);
            sleep_ms(5);
        }

        if (VL53L0X_GetRangingMeasurementData(pDev, &data) == VL53L0X_ERROR_NONE) {

            absolute_time_t now = get_absolute_time();
            uint64_t time_us = to_us_since_boot(now);
            uint32_t time_ms = time_us / 1000;

            if (data.RangeStatus == 0) {
                printf("%lu,%u\n", time_ms, data.RangeMilliMeter);
            } else {
                printf("Range error: %d\n", data.RangeStatus);
            }

            VL53L0X_ClearInterruptMask(
                pDev,
                VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY
            );
        }

        sleep_ms(50);
    }
}