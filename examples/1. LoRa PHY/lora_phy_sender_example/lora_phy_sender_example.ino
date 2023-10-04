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
 * This example demonstates sending a LoRa PHY packet every TX_INTERVAL seconds. 
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

#define TX_INTERVAL 15

rfthings_sx126x sx126x;
rft_status_t status;

char payload[255];
uint32_t payload_len;

String message = "hello world";

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

    Serial.begin(115200);

    while (!Serial && (millis() < 3000)) {}

    // Init SX126x
    Serial.println("#### SX126X INITIALIZE ####");
    status = sx126x.init(RFT_REGION_EU863_870);
    Serial.print("SX126x initialization: ");
    Serial.println(rft_status_to_str(status));
    Serial.print("Library version: ");
    Serial.println(LIBRARY_VERSION);

    // LoRa parameters
    sx126x.set_tx_power(14);
    sx126x.set_frequency(868100000);
    sx126x.set_spreading_factor(RFT_LORA_SPREADING_FACTOR_7);
    sx126x.set_bandwidth(RFT_LORA_BANDWIDTH_125KHZ);
    sx126x.set_coding_rate(RFT_LORA_CODING_RATE_4_5);
    sx126x.set_syncword(RFT_LORA_SYNCWORD_PUBLIC);
}

void loop(void)
{
    Serial.print("Sending LoRa PHY message: ");

    build_payload();
    status = sx126x.send_lora((byte *)payload, payload_len, 2000, sw_ctrl_set_mode_tx);
    Serial.println(rft_status_to_str(status));

    delay(TX_INTERVAL * 1000);
}

void build_payload(void)
{
    message.toCharArray(payload, 255);
    payload_len = message.length();
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