/*
 *     __          ____        _____                       __    _ __
 *    / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
 *   / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
 *  / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
 * /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
 *                              /_/
 * Author: mtnguyen
 * This library jointy developed by UCA & RFThings
 * 
 * This example demonstrates sending simple LoRaWAN packets in Activation by
 * Personalization (ABP) activation. The lorawan_abp_example send a "hello world"
 * packet every TX_INTERVAL seconds to the Gateway.
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
static uint8_t nwkS_key[] = {0xC8, 0x45, 0xC4, 0xD3, 0xBE, 0x42, 0xA6, 0xAA, 0xDA, 0xD9, 0x01, 0x97, 0xDD, 0x85, 0x35, 0x05};
static uint8_t appS_key[] = {0x4B, 0x58, 0x2A, 0xA2, 0x18, 0x8D, 0x3B, 0x98, 0xC9, 0xD3, 0x85, 0x5C, 0x1B, 0x2C, 0x30, 0x0F};
static uint8_t dev_addr[] = {0x26, 0x0B, 0x5E, 0x99};

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
    Serial.println("#### SX126X Initialize ####");
    status = sx126x.init(RFT_REGION_EU863_870);
    Serial.print("SX126x initialization: ");
    Serial.println(rft_status_to_str(status));
    Serial.print("Library version: ");
    Serial.println(LIBRARY_VERSION);

    // LoRaWAN parameters
    sx126x.set_lorawan_activation_type(RFT_LORAWAN_ACTIVATION_TYPE_ABP);
    sx126x.set_application_session_key(appS_key);
    sx126x.set_network_session_key(nwkS_key);
    sx126x.set_device_address(dev_addr);

    sx126x.set_tx_port(1);
    sx126x.set_rx1_delay(1000);

    // LoRa parameters
    sx126x.set_tx_power(22);
    sx126x.set_frequency(868100000);
    sx126x.set_spreading_factor(RFT_LORA_SPREADING_FACTOR_9);
    sx126x.set_bandwidth(RFT_LORA_BANDWIDTH_125KHZ);
    sx126x.set_coding_rate(RFT_LORA_CODING_RATE_4_6);
    sx126x.set_syncword(RFT_LORA_SYNCWORD_PUBLIC);
}

void loop(void)
{
    Serial.print("Sending LoRaWAN message: ");

    build_payload();
    status = sx126x.send_uplink((byte *)payload, payload_len, sw_ctrl_set_mode_tx, sw_ctrl_set_mode_rx);

    switch (status)
    {
    case RFT_STATUS_OK:
    {
        Serial.println("receive downlink packet");
        Serial.print("    RSSI: ");
        Serial.println(sx126x.get_rssi());
        Serial.print("    SNR: ");
        Serial.println(sx126x.get_snr());
        Serial.print("    Signal rssi: ");
        Serial.println(sx126x.get_signal_rssi());

        Serial.print("Downlink payload: ");
        for (int i = 0; i < sx126x.get_rx_length(); i++)
        {
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    break;
    case RFT_STATUS_TX_TIMEOUT:
    {
        Serial.println("Fail to send packet!");
    }
    break;
    case RFT_STATUS_RX_TIMEOUT:
    {
        Serial.println("TX Done, No downlink packet!");
    }
    break;
    default:
    {
        Serial.println(rft_status_to_str(status));
    }
    break;
    }

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
