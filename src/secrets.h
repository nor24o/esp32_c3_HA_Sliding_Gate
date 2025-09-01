#ifndef SECRETS_H
#define SECRETS_H

// --- WiFi & MQTT Credentials ---
#define MQTT_SERVER "192.168.1.100" // IP address of your MQTT broker
#define MQTT_USER "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"

// --- RF Remote Codes ---
// Replace these with the codes from your 433MHz remote
#define RF_GATE_OPEN_CODE   1234567
#define RF_GATE_CLOSE_CODE  7654321
#define RF_GATE_STOP_CODE   1111111
#define RF_GATE_POS50_CODE  2222222

#endif // SECRETS_H