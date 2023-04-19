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
#include <RTC.h>
#include <Wire.h>
#include <MicroNMEA.h> // http://librarymanager/All#MicroNMEA

#define I2C_GPS_QUECTEL_ADDRESS 0X10

#define TX_INTERVAL 10

// Keys and device address are MSB
static uint8_t nwkS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t appS_key[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t dev_addr[] = {0x00, 0x00, 0x00, 0x00};

rfthings_sx126x sx126x(E22_NSS, E22_NRST, E22_BUSY, E22_DIO1, E22_RXEN);
rft_status_t status;

uint32_t gnss_unix_time;
int32_t gnss_latitude = 0;
int32_t gnss_longitude = 0;
int32_t gnss_altitude = 0;
uint8_t gnss_num_sat = 0;

char payload[255];
uint32_t payload_len;

// GNSS
uint32_t current_timestamp = 0;
uint32_t gnss_data_get_timestamp = 0;
uint32_t gnss_info_print_timestamp = 0;
uint32_t lorawan_sending_timestamp = 0;
uint32_t quectelDelayTime = 0;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

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

    sx126x.set_tx_port(1);
    sx126x.set_rx1_delay(1000);

    // LoRa parameters
    sx126x.set_tx_power(14);
    sx126x.set_frequency(868100000);
    sx126x.set_spreading_factor(RFT_LORA_SPREADING_FACTOR_7);
    sx126x.set_bandwidth(RFT_LORA_BANDWIDTH_125KHZ);
    sx126x.set_coding_rate(RFT_LORA_CODING_RATE_4_5);
    sx126x.set_syncword(RFT_LORA_SYNCWORD_PUBLIC);

    // GNSS Init
    Wire.begin();

    pinMode(LS_GPS_ENABLE, OUTPUT);
    digitalWrite(LS_GPS_ENABLE, HIGH);
    pinMode(LS_GPS_V_BCKP, OUTPUT);
    digitalWrite(LS_GPS_V_BCKP, HIGH);

    digitalWrite(LS_VERSION_ENABLE, LOW);

    pinMode(SD_ON_OFF, OUTPUT);
    digitalWrite(SD_ON_OFF, HIGH);

    delay(100);
}

void loop()
{
    current_timestamp = millis();

    if ((current_timestamp - gnss_data_get_timestamp) > quectelDelayTime)
    {
        quectel_getData();
        gnss_data_get_timestamp = current_timestamp;
    }

    if ((current_timestamp - gnss_info_print_timestamp) > 5000)
    {
        gnss_info_print_timestamp = current_timestamp;

        gnss_latitude = nmea.getLatitude();
        gnss_longitude = nmea.getLongitude();
        nmea.getAltitude(gnss_altitude);
        gnss_num_sat = nmea.getNumSatellites();

        Serial.print("  GPS position: ");
        Serial.print(gnss_latitude / 1.0e6, 4);
        Serial.print(", ");
        Serial.print(gnss_longitude / 1.0e6, 4);
        Serial.print(" alt: ");
        Serial.print(gnss_altitude / 1.0e3, 2);
        Serial.print(" (");
        Serial.print(gnss_num_sat);
        Serial.println(" satellites)");

        Serial.print("Valid fix: ");
        Serial.println(nmea.isValid() ? "yes" : "no");

        Serial.print("Nav. system: ");
        if (nmea.getNavSystem())
            Serial.println(nmea.getNavSystem());
        else
            Serial.println("none");

        Serial.print("HDOP: ");
        Serial.println(nmea.getHDOP() / 10., 1);

        Serial.print("  GPS time: ");
        Serial.print(nmea.getYear());
        Serial.print("-");
        Serial.print(nmea.getMonth());
        Serial.print("-");
        Serial.print(nmea.getDay());
        Serial.print(" ");
        Serial.print(nmea.getHour());
        Serial.print(":");
        Serial.print(nmea.getMinute());
        Serial.print(":");
        Serial.println(nmea.getSecond());

        gnss_unix_time = unixTimestamp(nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond());
        Serial.print("  Unix time GPS: ");
        Serial.println(gnss_unix_time);
    }

    if ((current_timestamp - lorawan_sending_timestamp) > 15000)
    {
        lorawan_sending_timestamp = current_timestamp;

        digitalWrite(LED_BUILTIN, HIGH);
        delay(125);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(125);
        digitalWrite(LED_BUILTIN, LOW);

        payload[0] = (gnss_unix_time >> 0) & 0xff;
        payload[1] = (gnss_unix_time >> 8) & 0xff;
        payload[2] = (gnss_unix_time >> 16) & 0xff;
        payload[3] = (gnss_unix_time >> 24) & 0xff;

        payload[4] = (gnss_latitude >> 0) & 0xff;
        payload[5] = (gnss_latitude >> 8) & 0xff;
        payload[6] = (gnss_latitude >> 16) & 0xff;
        payload[7] = (gnss_latitude >> 24) & 0xff;

        payload[8] = (gnss_longitude >> 0) & 0xff;
        payload[9] = (gnss_longitude >> 8) & 0xff;
        payload[10] = (gnss_longitude >> 16) & 0xff;
        payload[11] = (gnss_longitude >> 24) & 0xff;

        payload[12] = (gnss_altitude >> 0) & 0xff;
        payload[13] = (gnss_altitude >> 8) & 0xff;
        payload[14] = (gnss_altitude >> 16) & 0xff;
        payload[15] = (gnss_altitude >> 24) & 0xff;

        payload[16] = gnss_num_sat & 0xff;

        payload_len = 17;

        // Send LoRaWAN packet
        Serial.print("Sending LoRaWAN Packet: ");
        status = sx126x.send_uplink((byte *)payload, payload_len, NULL, NULL);
        Serial.println(rft_status_to_str(status));
    }
}

unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec)
{
    const short days_since_beginning_of_year[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int leap_years = ((year - 1) - 1968) / 4 - ((year - 1) - 1900) / 100 + ((year - 1) - 1600) / 400;
    long days_since_1970 = (year - 1970) * 365 + leap_years + days_since_beginning_of_year[month - 1] + day - 1;
    if ((month > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
        days_since_1970 += 1; /* +leap day, if year is a leap year */
    return sec + 60 * (min + 60 * (hour + 24 * days_since_1970));
}

void quectel_getData(void)
{
    char revChar = 0;

    Wire.requestFrom(I2C_GPS_QUECTEL_ADDRESS, 255);
    while (Wire.available())
    {
        revChar = Wire.read();
        nmea.process(revChar);
        Serial.print(revChar);

        if (revChar == 0x0A) // End character | Garbage bytes
        {
            quectelDelayTime = 500; // 500ms
            return;
        }
    }
    quectelDelayTime = 5; // 5ms (Minimum delay time for slave to prepare the next NMEA packet is 2ms)
}
