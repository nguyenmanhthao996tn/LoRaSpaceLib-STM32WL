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
 * This sketch operates the same as the lr_fhss_raw_example sketch.
 * Except for the LoRaWAN packet format. Thus, it requires Device address (dev_addr),
 * Network Session Key (NwkSKey) and Application Session Key (AppSKey) to operate.
 * Replace those information in the source code with the ones received from your
 * LoRaWAN Network Server. Make sure that they are all in MSB format.
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
static uint8_t nwkS_key[] = {0x28, 0x81, 0x41, 0x5c, 0x1c, 0xd8, 0xc1, 0xc9, 0x27, 0x4e, 0xa4, 0x24, 0x09, 0xa8, 0x28, 0xa3};
static uint8_t appS_key[] = {0xd2, 0xe6, 0xbf, 0x00, 0x9e, 0x16, 0x33, 0x3d, 0x49, 0x85, 0x4c, 0x61, 0xbf, 0x62, 0xa2, 0xce};
static uint8_t dev_addr[] = {0x01, 0x44, 0x1c, 0x5f};

rfthings_sx126x sx126x;
rft_status_t status;

char payload[255];
uint32_t payload_len;

const String message = "Hello Space!";

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

    // LoRaWAN parameters
    sx126x.set_lorawan_activation_type(RFT_LORAWAN_ACTIVATION_TYPE_ABP);
    sx126x.set_application_session_key(appS_key);
    sx126x.set_network_session_key(nwkS_key);
    sx126x.set_device_address(dev_addr);

    // Config LR-FHSS parameter
    sx126x.set_lrfhss_codingRate(RFT_LRFHSS_CODING_RATE_1_3);
    sx126x.set_lrfhss_bandwidth(RFT_LRFHSS_BANDWIDTH_136_7_KHZ);
    sx126x.set_lrfhss_grid(RFT_LRFHSS_GRID_3_9_KHZ);
    sx126x.set_lrfhss_hopping(true);
    sx126x.set_lrfhss_nbSync(4);
    sx126x.set_lrfhss_frequency(868200000);
    sx126x.set_lrfhss_power(22);
    sx126x.set_lrfhss_syncword(0x2C0F7995);
}

void loop(void)
{
    Serial.println("Sending LR-FHSS message");
    build_payload();
    status = sx126x.send_lorawan_over_lrfhss((byte *)payload, message.length(), sw_ctrl_set_mode_tx);
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
