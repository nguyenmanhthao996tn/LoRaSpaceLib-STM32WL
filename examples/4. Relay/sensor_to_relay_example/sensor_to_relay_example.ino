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
 * This example sends a "Hello world" LoRaWAN packet with 1-second
 * extended preamble length every TX_INTERVAL seconds.
 * 
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

// Keys and device address are MSB
static uint8_t nwkS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t appS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t dev_addr[] = {0x01, 0x02, 0x03, 0x04};

rfthings_sx126x sx126x;
rft_status_t status;

char payload[255];
uint32_t payload_len;

String message = "Hello world";

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
  Serial.println("Sensor-to-Relay example");

  // Inititialize the LoRa Module SX126x
  status = sx126x.init(RFT_REGION_EU863_870);
  Serial.print("LoRa SX126x: ");
  Serial.println(rft_status_to_str(status));

  // set up LoRaWAN key (ABP by default)
  sx126x.set_lorawan_activation_type(RFT_LORAWAN_ACTIVATION_TYPE_ABP);
  sx126x.set_application_session_key(appS_key);
  sx126x.set_network_session_key(nwkS_key);
  sx126x.set_device_address(dev_addr);

  // Set a custom LoRaWAN configuration
  sx126x.set_send_to_relay(true);
  sx126x.set_frequency(866600000);
  sx126x.set_spreading_factor(RFT_LORA_SPREADING_FACTOR_7);
  sx126x.set_bandwidth(RFT_LORA_BANDWIDTH_125KHZ);
  sx126x.set_coding_rate(RFT_LORA_CODING_RATE_4_5);
  sx126x.set_syncword(RFT_LORA_SYNCWORD_PUBLIC);
  sx126x.set_tx_power(14);

  delay(2000);
}

void loop(void)
{
  Serial.print("Sending LoRaWAN message: ");

  build_payload();
  status = sx126x.send_uplink((byte *)payload, payload_len, sw_ctrl_set_mode_tx, sw_ctrl_set_mode_rx);
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
