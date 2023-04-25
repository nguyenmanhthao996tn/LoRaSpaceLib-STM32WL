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

#include <RFThings.h>
#include <radio/sx126x/rfthings_sx126x.h>

// Device information (For uploading relay status)
static uint8_t nwkS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t appS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE
static uint8_t dev_addr[] = {0x01, 0x02, 0x03, 0x04};                                                                         // <-- MODIFY THIS INFORMATION ACCORDING TO YOUR USECASE

rfthings_sx126x sx126x;
rft_status_t status;

// Relay params (Listen to all devices meet this LoRaPHY Params, NO FILTER)
rft_lora_params_t relay_lora_params;
byte payload[255];
uint32_t payload_len;

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

  // Inititialize the LoRa Module SX126x
  status = sx126x.init(RFT_REGION_EU863_870);
  Serial.print("LoRa SX126x: ");
  Serial.println(rft_status_to_str(status));

  // Set up LoRaWAN key (ABP by default)
  sx126x.set_lorawan_activation_type(RFT_LORAWAN_ACTIVATION_TYPE_ABP);
  sx126x.set_application_session_key(appS_key);
  sx126x.set_network_session_key(nwkS_key);
  sx126x.set_device_address(dev_addr);
  sx126x.set_rx1_delay(1000);

  // Setting Relay LoRa Params
  relay_lora_params.frequency = 866600000;
  relay_lora_params.spreading_factor = RFT_LORA_SPREADING_FACTOR_7;
  relay_lora_params.bandwidth = RFT_LORA_BANDWIDTH_125KHZ;
  relay_lora_params.coding_rate = RFT_LORA_CODING_RATE_4_5;
  relay_lora_params.syncword = RFT_LORA_SYNCWORD_PUBLIC;
  relay_lora_params.relay_sleep_interval_us = 1E6; // 1 second
  relay_lora_params.relay_rx_symbol = 5;
  relay_lora_params.relay_max_rx_packet_length = 120;

  delay(2000);
}

void loop(void)
{
  switch (sx126x.relay(&relay_lora_params, payload, payload_len, NULL, NULL))
  {
  case RFT_STATUS_OK:
    if (payload_len > 0)
    {
      // TODO: Filter packet and verify packet integrity

      Serial.print("[Relay] OK: Received ");
      Serial.print(payload_len);
      Serial.print(" bytes from Device Address: 0x");
      Serial.println(parse_dev_id(payload, payload_len), HEX);

      Serial.print("RAW Payload: ");
      for (uint8_t i = 0; i < payload_len; i++)
      {
        Serial.print(payload[i]);
        Serial.print(" ");
      }
      Serial.println();

      Serial.print("    Received RSSI: ");
      Serial.println(relay_lora_params.rssi);
      Serial.print("    Received SNR: ");
      Serial.println(relay_lora_params.snr);
      Serial.print("    Received Signal RSSI: ");
      Serial.println(relay_lora_params.signal_rssi);
    }
    else
    {
      Serial.println("[Relay] ERROR: Unknown ERROR!");
    }
    break;
  case RFT_STATUS_RX_TIMEOUT:
    Serial.println("[Relay] ERROR: Preamble detected, RX timeout!");
    break;
  case RFT_STATUS_PREAMBLE_DETECT_FAIL:
    Serial.println("[Relay] INFO: Nothing to relay/receive!");
    break;
  default:
    break;
  }
}

uint32_t parse_dev_id(byte *payload, uint8_t len)
{
  if (payload == NULL)
    return 0;
  if (len < 11)
    return 0; // ToDo: Verify the minimum size of a LoRaWAN Packet

  return ((payload[1]) | (payload[2] << 8) | (payload[3] << 16) | (payload[4] << 24));
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
