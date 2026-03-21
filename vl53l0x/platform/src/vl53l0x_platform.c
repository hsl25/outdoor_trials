#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define VL53L0X_I2C_PORT i2c0

VL53L0X_Error VL53L0X_WriteMulti(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint8_t *pdata,
    uint32_t count
) {
    uint8_t buffer[count + 1];
    buffer[0] = index;

    for (uint32_t i = 0; i < count; i++) {
        buffer[i + 1] = pdata[i];
    }

    int ret = i2c_write_blocking(
        VL53L0X_I2C_PORT,
        Dev->I2cDevAddr,
        buffer,
        count + 1,
        false
    );

    return (ret < 0) ? VL53L0X_ERROR_CONTROL_INTERFACE : VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint8_t *pdata,
    uint32_t count
) {
    int ret = i2c_write_blocking(
        VL53L0X_I2C_PORT,
        Dev->I2cDevAddr,
        &index,
        1,
        true
    );

    if (ret < 0)
        return VL53L0X_ERROR_CONTROL_INTERFACE;

    ret = i2c_read_blocking(
        VL53L0X_I2C_PORT,
        Dev->I2cDevAddr,
        pdata,
        count,
        false
    );

    return (ret < 0) ? VL53L0X_ERROR_CONTROL_INTERFACE : VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint8_t data
) {
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdByte(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint8_t *data
) {
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    sleep_ms(5);
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint16_t data)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(data >> 8);     // MSB
    buffer[1] = (uint8_t)(data & 0xFF);   // LSB

    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_RdWord(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint16_t *data)
{
    uint8_t buffer[2];
    VL53L0X_Error status;

    status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    if (status != VL53L0X_ERROR_NONE)
        return status;

    *data = ((uint16_t)buffer[0] << 8) | buffer[1];

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrDWord(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint32_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data);

    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdDWord(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint32_t *data)
{
    uint8_t buffer[4];
    VL53L0X_Error status;

    status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    if (status != VL53L0X_ERROR_NONE)
        return status;

    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
             (uint32_t)buffer[3];

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UpdateByte(
    VL53L0X_DEV Dev,
    uint8_t index,
    uint8_t and_mask,
    uint8_t or_mask)
{
    uint8_t data;
    VL53L0X_Error status;

    status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE)
        return status;

    data = (data & and_mask) | or_mask;

    return VL53L0X_WrByte(Dev, index, data);
}

