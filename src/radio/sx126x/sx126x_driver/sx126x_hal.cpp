/*
 *     __          ____        _____                       __    _ __
 *    / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
 *   / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
 *  / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
 * /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
 *                              /_/
 * Author: m1nhle, mtnguyen
 * Lib jointy developed by UCA & RFThings
 */

#include "Arduino.h"
#include "SPI.h"
#include "sx126x_hal.h"
#include <SubGhz.h>

// #define __DEBUG_SPI_CMD_PRINT_CMD__
// #define __DEBUG_SPI_CMD_DISABLE_TX_CMD__

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length)
{
    uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memcpy(buffer + command_length, data, data_length);

#if !defined(__DEBUG_SPI_CMD_DISABLE_TX_CMD__)
    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);
    while (SubGhz.isBusy())
        ;

    for (uint16_t i = 0; i < (command_length + data_length); i++)
    {
        SubGhz.SPI.transfer(buffer[i]);
    }

    SubGhz.setNssActive(false);
    SubGhz.SPI.endTransaction();
#endif

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    Serial.printf("cmd> sx126x_hal_write: ");
    for (uint8_t i = 0; i < (command_length + data_length); i++)
    {
        Serial.printf(" %.2x", buffer[i]);
    }
    Serial.println();
#endif

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write_bulk(const void *context, const uint8_t *command, const uint16_t command_length,
                                          const uint8_t *data, const uint16_t data_length)
{
    uint8_t buffer[32];
    uint8_t *ptr = (uint8_t *)data;
    uint16_t _data_length = data_length;

#if !defined(__DEBUG_SPI_CMD_DISABLE_TX_CMD__)
    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);
    while (SubGhz.isBusy())
        ;

    for (uint8_t i = 0; i < command_length; i++)
    {
        SubGhz.SPI.transfer(command[i]);
    }

    while (_data_length > 0)
    {
        uint8_t chunkSize = ((_data_length) < (sizeof buffer) ? (_data_length) : (sizeof buffer)); // MIN(size, sizeof buffer);
        memcpy(buffer, ptr, chunkSize);

        for (uint8_t i = 0; i < chunkSize; i++)
        {
            SubGhz.SPI.transfer(buffer[i]);
        }

        _data_length -= chunkSize;
        ptr += chunkSize;
    }

    SubGhz.setNssActive(false);
    SubGhz.SPI.endTransaction();
#endif

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    // uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memcpy(buffer + command_length, data, data_length);
    Serial.printf("cmd> sx126x_hal_write_bulk: ");
    for (uint8_t i = 0; i < (command_length + data_length); i++)
    {
        Serial.printf(" %.2x", buffer[i]);
    }
    Serial.println();
#endif

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length)
{
    uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memset(buffer + command_length, SX126X_NOP, data_length);

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    Serial.printf("cmd> sx126x_hal_read: \n            Tx: ");
    for (uint8_t i = 0; i < (command_length + data_length); i++)
    {
        Serial.printf(" %.2x", buffer[i]);
    }
    Serial.println();
#endif

#if !defined(__DEBUG_SPI_CMD_DISABLE_TX_CMD__)
    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);
    while (SubGhz.isBusy())
        ;

    for (uint16_t i = 0; i < (command_length + data_length); i++)
    {
        buffer[i] = SubGhz.SPI.transfer(buffer[i]);
    }

    SubGhz.setNssActive(false);
    SubGhz.SPI.endTransaction();
#endif

    memcpy(data, buffer + command_length, data_length);

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    Serial.printf("            Rx: ");
    for (uint8_t i = 0; i < (command_length + data_length); i++)
    {
        Serial.printf(" %.2x", buffer[i]);
    }
    Serial.println();
#endif

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
#if !defined(__DEBUG_SPI_CMD_DISABLE_TX_CMD__)
    delay(20);
    SubGhz.setResetActive(true);
    delay(100);
    SubGhz.setResetActive(false);
    delay(20);
#endif

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    Serial.printf("cmd> sx126x_hal_reset\n");
#endif

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
#if !defined(__DEBUG_SPI_CMD_DISABLE_TX_CMD__)
    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);
    delay(5);

    SubGhz.SPI.transfer(0x00);

    SubGhz.setNssActive(false);
    SubGhz.SPI.endTransaction();

    while (SubGhz.isBusy())
        ;
#endif

#if (defined(__DEBUG_SPI_CMD_PRINT_CMD__) && defined(USE_LOW_POWER_FEATURE_WITH_SLEEP))
    Serial.printf("cmd> sx126x_hal_wakeup\n");
#endif

    return SX126X_HAL_STATUS_OK;
}