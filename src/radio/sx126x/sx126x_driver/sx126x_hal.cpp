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

#define __DEBUG_SPI_COMMAND__

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length)
{
    //     sx126x_hal_t* sx126x_hal = ( sx126x_hal_t* ) context;

    //     while(digitalRead(sx126x_hal->busy)) { }

    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    // #endif
    //     digitalWrite(sx126x_hal->nss, LOW);

    // uint8_t buffer[255];
    // memcpy(buffer, command, command_length);
    // memcpy(buffer + command_length, data, data_length);

    //     SPI.transfer(buffer, (uint32_t)(command_length + data_length));

    //     digitalWrite(sx126x_hal->nss, HIGH);
    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.endTransaction();
    // #endif

    uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memcpy(buffer + command_length, data, data_length);

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

#if defined(__DEBUG_SPI_COMMAND__)
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
    sx126x_hal_t *sx126x_hal = (sx126x_hal_t *)context;

    //     while(digitalRead(sx126x_hal->busy)) { }

    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    // #endif
    //     digitalWrite(sx126x_hal->nss, LOW);

    //     SPI.transfer((uint8_t* )command, command_length);

    //     uint8_t buffer[32];
    // 	uint8_t *ptr = (uint8_t*)data;
    //     uint16_t _data_length = data_length;
    // 	while(_data_length) {
    // 		uint8_t chunkSize = ((_data_length) < (sizeof buffer) ? (_data_length) : (sizeof buffer)); // MIN(size, sizeof buffer);
    // 		memcpy(buffer, ptr, chunkSize);
    // 		SPI.transfer(buffer, chunkSize);
    // 		_data_length -= chunkSize;
    // 		ptr += chunkSize;
    // 	}

    //     digitalWrite(sx126x_hal->nss, HIGH);
    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.endTransaction();
    // #endif

    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);
    while (SubGhz.isBusy())
        ;

    for (uint8_t i = 0; i < command_length; i++)
    {
        SubGhz.SPI.transfer(command[i]);
    }

    uint8_t buffer[32];
    uint8_t *ptr = (uint8_t *)data;
    uint16_t _data_length = data_length;
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

#if defined(__DEBUG_SPI_COMMAND__)
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
    sx126x_hal_t *sx126x_hal = (sx126x_hal_t *)context;

    //     while(digitalRead(sx126x_hal->busy)) { }

    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    // #endif
    //     digitalWrite(sx126x_hal->nss, LOW);

    //     uint8_t buffer[255];
    //     memcpy(buffer, command, command_length);
    //     memset(buffer + command_length, SX126X_NOP, data_length);

    //     SPI.transfer(buffer, (uint32_t)(command_length + data_length));
    //     memcpy(data, buffer + command_length, data_length);

    //     digitalWrite(sx126x_hal->nss, HIGH);
    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.endTransaction();
    // #endif

    uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memset(buffer + command_length, SX126X_NOP, data_length);

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

    memcpy(data, buffer + command_length, data_length);

#if defined(__DEBUG_SPI_COMMAND__)
    // uint8_t buffer[255];
    memcpy(buffer, command, command_length);
    memcpy(buffer + command_length, data, data_length);
    Serial.printf("cmd> sx126x_hal_read: ");
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
    sx126x_hal_t *sx126x_hal = (sx126x_hal_t *)context;

    // delay(20);
    // digitalWrite(sx126x_hal->reset, LOW);
    // delay(50);
    // digitalWrite(sx126x_hal->reset, HIGH);
    // delay(50);

    delay(20);
    SubGhz.setResetActive(true);
    delay(50);
    SubGhz.setResetActive(false);
    delay(20);

#if defined(__DEBUG_SPI_COMMAND__)
    Serial.printf("cmd> sx126x_hal_reset\n");
#endif

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    sx126x_hal_t *sx126x_hal = (sx126x_hal_t *)context;

    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    // #endif
    //     digitalWrite(sx126x_hal->nss, LOW);

    //     SPI.transfer(SX126X_NOP, 1);

    // 	digitalWrite(sx126x_hal->nss, HIGH);
    // #ifdef SPI_HAS_TRANSACTION
    //     SPI.endTransaction();
    // #endif

    //     while (digitalRead(sx126x_hal->busy)) { }

    SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
    SubGhz.setNssActive(true);

    SubGhz.SPI.transfer(0x00);

    SubGhz.setNssActive(false);
    SubGhz.SPI.endTransaction();

    while (SubGhz.isBusy())
        ;

#if defined(__DEBUG_SPI_COMMAND__)
    Serial.printf("cmd> sx126x_hal_wakeup\n");
#endif

    return SX126X_HAL_STATUS_OK;
}