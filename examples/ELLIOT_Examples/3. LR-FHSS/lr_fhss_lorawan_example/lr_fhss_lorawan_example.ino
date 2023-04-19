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

#define TX_INTERVAL 10

// Keys and device address are MSB
static uint8_t nwkS_key[] = {0xC8, 0x45, 0xC4, 0xD3, 0xBE, 0x42, 0xA6, 0xAA, 0xDA, 0xD9, 0x01, 0x97, 0xDD, 0x85, 0x35, 0x05};
static uint8_t appS_key[] = {0x4B, 0x58, 0x2A, 0xA2, 0x18, 0x8D, 0x3B, 0x98, 0xC9, 0xD3, 0x85, 0x5C, 0x1B, 0x2C, 0x30, 0x0F};
static uint8_t dev_addr[] = {0x26, 0x0B, 0x5E, 0x99};

rfthings_sx126x sx126x(-1, -1, -1, -1, -1);
rft_status_t status;

char payload[255];
uint32_t payload_len;

const String message = "Hello Space!";

void setup()
{
    Serial.begin(115200);

    while (!Serial && (millis() < 3000))
        ;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Init SX126x
    Serial.println("#### SX126X INITIALIZE ####");
    status = sx126x.init(RFT_REGION_EU863_870);
    Serial.print("SX126x initialization: ");
    Serial.println(rft_status_to_str(status));

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
    sx126x.set_lrfhss_frequency(862000000);
    sx126x.set_lrfhss_power(21);
}

void loop()
{
    Serial.println("Sending LR-FHSS message");
    message.toCharArray(payload, 255);
    status = sx126x.send_lorawan_over_lrfhss((byte *)payload, message.length());
    Serial.println(rft_status_to_str(status));

    delay(TX_INTERVAL * 1000);
}

void build_payload(void)
{
    message.toCharArray(payload, 255);
    payload_len = message.length();
}