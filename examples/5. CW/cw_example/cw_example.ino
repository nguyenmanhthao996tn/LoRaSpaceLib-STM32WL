/*
 *     __          ____        _____                       __    _ __
 *    / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
 *   / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
 *  / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
 * /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
 *                              /_/
 * Author: m1nhle, mtnguyen
 * Lib jointy developed by UCA & RFThings
 * 
 * This section is intended for antenna or system functional testing.
 * The sketch in this section generates a continuous wave at a specific
 * Frequency and Output power level.
 */

/* Support REGION
    RFT_REGION_EU863_870
    RFT_REGION_US902_928
    RFT_REGION_CN470_510
    RFT_REGION_AU915_928
    RFT_REGION_AS920_923
    RFT_REGION_AS923_925
    RFT_REGION_KR920_923
    RFT_REGION_IN865_867
*/

#include <RFThings.h>
#include <radio/sx126x/rfthings_sx126x.h>

rfthings_sx126x sx126x;
rft_status_t status;

#define SW_VCTL1_PIN PB8
#define SW_VCTL2_PIN PC13

typedef enum {
    RF_SW_MODE_TX = 0,
    RF_SW_MODE_RX
} rf_sw_mode_t;

void setup(void)
{
    pinMode(SW_VCTL1_PIN, OUTPUT);
    pinMode(SW_VCTL2_PIN, OUTPUT);
    sw_ctrl_set_mode(RF_SW_MODE_TX);

    Serial.begin(115200);

    while (!Serial && (millis() < 3000)) {}

    // Init SX126x
    Serial.println("#### SX126X INITIALIZE ####");
    status = sx126x.init(RFT_REGION_EU863_870);
    Serial.print("SX126x initialization: ");
    Serial.println(rft_status_to_str(status));
    Serial.print("Library version: ");
    Serial.println(LIBRARY_VERSION);

    // CW Configuration
    sx126x.set_frequency(868100000);
    sx126x.set_tx_power(22);
}

void loop(void)
{
    Serial.println("start_continuous_wave");
    sw_ctrl_set_mode_tx();
    sx126x.start_continuous_wave();
    delay(5000); // 5s

    Serial.println("stop_continuous_wave");
    sx126x.stop_continuous_wave();
    delay(1000); // 1s
}

void sw_ctrl_set_mode(rf_sw_mode_t mode)
{
  if (mode == RF_SW_MODE_TX)
  {
    digitalWrite(SW_VCTL1_PIN, LOW);
    digitalWrite(SW_VCTL2_PIN, HIGH);
  }
  else
  {
    digitalWrite(SW_VCTL1_PIN, HIGH);
    digitalWrite(SW_VCTL2_PIN, LOW);
  }
}

void sw_ctrl_set_mode_tx(void)
{
    sw_ctrl_set_mode(RF_SW_MODE_TX);
}

void sw_ctrl_set_mode_rx(void)
{
    sw_ctrl_set_mode(RF_SW_MODE_RX);
}
