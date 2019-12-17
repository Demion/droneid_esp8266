#include <ESP8266WiFi.h>
#include <string.h>
#include <math.h>

#define DEG_TO_RAD(DEG) ((DEG) * M_PI / 180.0f)
#define RAD_TO_DEG(RAD) ((RAD) * 180.0f / M_PI)

uint8_t wifi_beacon[] =
{
    0x80,                                                   // Version Type/Subtype
    0x00,                                                   // Flags
    0x00, 0x00,                                             // Duration
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,                     // Destination address
    0x60, 0x60, 0x1f, 0x00, 0x00, 0x00,                     // Source address
    0x60, 0x60, 0x1f, 0x00, 0x00, 0x00,                     // BSS Id
    0x00, 0x00,                                             // Fragment Sequence
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,         // Timestamp
    0x64, 0x00,                                             // Beacon Interval
    0x31, 0x04,                                             // Capabilities
    // SSID
    0x00, 0x0c, 0x4d, 0x61, 0x76, 0x69, 0x63, 0x2d, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
    // Supported Rates
    0x01, 0x08, 0x82, 0x84, 0x8b, 0x0c, 0x12, 0x96, 0x18, 0x24,
    // Current Channel
    0x03, 0x01, 0x01,
    // Traffic Indication Map
    0x05, 0x04, 0x00, 0x01, 0x00, 0x00,
    // Country Information
    0x07, 0x06, 0x55, 0x53, 0x00, 0x01, 0x0b, 0x1e,
    // ERP Information
    0x2a, 0x01, 0x00,
    // Extended Supported Rates
    0x32, 0x04, 0x30, 0x48, 0x60, 0x6c,
    // HT Capabilities
    0x2d, 0x1a, 0xac, 0x01, 0x02, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // HT Information
    0x3d, 0x16, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // RSN Information
    0x30, 0x14, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x04, 0x01, 0x00, 0x00, 0x0f, 0xac, 0x02, 0x0c, 0x00,
    // Vendor Specific: Microsoft Corp.: WMM/WME: Parameter Element
    0xdd, 0x18, 0x00, 0x50, 0xf2, 0x02, 0x01, 0x01, 0x00, 0x00, 0x03, 0xa4, 0x00, 0x00, 0x27, 0xa4, 0x00, 0x00, 0x42, 0x43, 0x5e, 0x00, 0x62, 0x32, 0x2f, 0x00,
};

struct droneid_header
{
    uint8_t tag_number;
    uint8_t tag_length;

    uint8_t oui[3];

    uint8_t vendor_type;
    uint8_t unk1;
    uint8_t unk2;
} __attribute__((packed));

struct droneid_flight_reg
{
    droneid_header header;

    uint8_t sub_cmd;
    uint8_t ver;
    uint16_t seq;
    uint16_t state_info;
    char sn[16];
    int32_t longitude;
    int32_t latitude;
    int16_t altitude;
    int16_t height;
    int16_t v_north;
    int16_t v_east;
    int16_t v_up;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int32_t longitude_home;
    int32_t latitude_home;
    uint8_t product_type;
    uint8_t uuid_len;
    char uuid[20];
} __attribute__((packed));

struct droneid_flight_purpose
{
    droneid_header header;

    uint8_t sub_cmd;
    char sn[16];
    uint8_t drone_id_len;
    char drone_id[10];
    uint8_t purpose_len;
    char purpose[100];
} __attribute__((packed));

#define SOURCE_ADDRESS 10
#define BSSID_ADDRESS 16
#define SSID 38
#define CURRENT_CHANNEL 62
#define HT_INFO_CHANNEL 116

int count = 1;

uint64_t macs[] = {0x0100001f6060, 0x0200001f6060, 0x0300001f6060, 0x0400001f6060};
const char *ssids[] = {"Mavic-000001", "Mavic-000002", "Spark-000003", "Spark-000004"};
uint8_t channels[] = {1, 2, 3, 4};

uint16_t seq = 0;

void update_wifi_beacon(uint64_t mac, const char *ssid, uint8_t channel)
{
    memcpy(&wifi_beacon[SOURCE_ADDRESS], &mac, 6);
    memcpy(&wifi_beacon[BSSID_ADDRESS], &mac, 6);

    memcpy(&wifi_beacon[SSID], ssid, 12);

    wifi_beacon[CURRENT_CHANNEL] = channel;
    wifi_beacon[HT_INFO_CHANNEL] = channel;
}

void update_droneid_header(droneid_header *header, uint8_t length)
{
    header->tag_number = 0xdd;
    header->tag_length = length - 2;

    header->oui[0] = 0x26;
    header->oui[1] = 0x37;
    header->oui[2] = 0x12;

    header->vendor_type = 0x58;
    header->unk1 = 0x62;
    header->unk2 = 0x13;
}

int wifi_send_beacon(uint64_t mac, const char *ssid, uint8_t channel)
{
    wifi_set_channel(channel);

    update_wifi_beacon(mac, ssid, channel);

    return wifi_send_pkt_freedom(wifi_beacon, sizeof(wifi_beacon), true);
}

int wifi_send_droneid(uint64_t mac, const char *ssid, uint8_t channel, void *packet, uint8_t length)
{
    wifi_set_channel(channel);

    update_wifi_beacon(mac, ssid, channel);
    update_droneid_header((droneid_header*) packet, length);

    uint8_t wifi_droneid[512];

    memcpy(wifi_droneid, wifi_beacon, sizeof(wifi_beacon));
    memcpy(wifi_droneid + sizeof(wifi_beacon), packet, length);

    return wifi_send_pkt_freedom(wifi_droneid, sizeof(wifi_beacon) + length, true);
}

void setup()
{
    Serial.begin(115200);

    delay(500);

    wifi_set_opmode(STATION_MODE);
    wifi_promiscuous_enable(1);
}

void loop()
{
    for (int i = 0; i < count; ++i)
    {
        int result = 0;

        droneid_flight_reg flight_reg = {0};

        flight_reg.sub_cmd = 0x10;
        flight_reg.ver = 0x01;
        flight_reg.seq = seq;
        flight_reg.state_info = 0x0fff;
        strncpy(flight_reg.sn, "0123456789ABCD", 14);
        flight_reg.longitude = (int32_t) (DEG_TO_RAD(180.0f) * 10000000.0f);
        flight_reg.latitude = (int32_t) (DEG_TO_RAD(90.0f) * 10000000.0f);
        flight_reg.altitude = 200;
        flight_reg.height = 100;
        flight_reg.v_north = 10;
        flight_reg.v_east = 20;
        flight_reg.v_up = 30;
        flight_reg.pitch = (int16_t) (40.0f * 100.0f);
        flight_reg.roll = (int16_t) (50.0f * 100.0f);
        flight_reg.yaw = (int16_t) (60.0f * 100.0f);
        flight_reg.longitude_home = (int32_t) (DEG_TO_RAD(90.0f) * 10000000.0f);
        flight_reg.latitude_home = (int32_t) (DEG_TO_RAD(45.0f) * 10000000.0f);
        flight_reg.product_type = 0x10;
        flight_reg.uuid_len = 18;
        strncpy(flight_reg.uuid, "123456789123456789", 18);

        result = wifi_send_droneid(macs[i], ssids[i], channels[i], &flight_reg, sizeof(flight_reg));

        Serial.print(i);
        Serial.print("> wifi_send_droneid_flight_reg = ");
        Serial.println(result);

        delay(10);

        droneid_flight_purpose flight_purpose = {0};

        flight_purpose.sub_cmd = 0x11;
        strncpy(flight_purpose.sn, "0123456789ABCD", 14);
        //flight_purpose.drone_id_len = 10;
        //strncpy(flight_purpose.drone_id, "0123456789", 10);
        //flight_purpose.purpose_len = 4;
        //strncpy(flight_purpose.purpose, "TEST", 4);

        result = wifi_send_droneid(macs[i], ssids[i], channels[i], &flight_purpose, sizeof(flight_purpose));

        Serial.print(i);
        Serial.print("> wifi_send_droneid_flight_purpose = ");
        Serial.println(result);

        delay(10);
    }

    ++seq;

    delay(100);
}
