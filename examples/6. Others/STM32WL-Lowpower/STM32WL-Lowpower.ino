/*
 *     __          ____        _____                       __    _ __
 *    / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
 *   / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
 *  / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
 * /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
 *                              /_/
 * Author: mtnguyen
 * Lib jointy developed by UCA & RFThings
 *
 * This example demonstrates how to set the MCU/Board in Sleep mode and waking up with RTC alarm.
 * Please install these library to proceed.
 * 
 * STM32LowPower: https://github.com/stm32duino/STM32LowPower
 * STM32RTC: https://github.com/stm32duino/STM32RTC
 * 
 */

#include <STM32LowPower.h> // https://github.com/stm32duino/STM32LowPower
#include <STM32RTC.h>      // https://github.com/stm32duino/STM32RTC

STM32RTC &rtc = STM32RTC::getInstance();

void setup(void)
{
  rtc.begin();

  Serial.begin(115200);
  while (!Serial)
  {
  }
  Serial.print("STM32WL Low Power + RTC Example");

  LowPower.begin();
  LowPower.enableWakeupFrom(&rtc, alarmMatch);
}

void loop(void)
{
  Serial.println("Loop");
  Serial.flush();

  delay(2000); // Stay awake for 2 seconds

  rtc.setAlarmEpoch(rtc.getEpoch() + 5); // Set an alarm to wake up after 5 seconds
  LowPower.deepSleep();
}

void alarmMatch(void *data)
{
  // Do nothing here
}