/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
   - adapted by A.Combes ISIS 26.1.2025, also inspired by https://wolles-elektronikkiste.de/en/esp-now
*********/
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>
#include <string> // for string and to_string()
using namespace std;

// TODO : update if you have more than one mote (sink does not count)
#define MOTE_COUNT 1

// Put here your network credentials (Wifi Access Point)
const char *ssid = "iot";
const char *password = "iotisis;";

// TODO : Add your MQTT Broker IP address, example:
// const char* mqtt_server = "192.168.3.xx";
const char *mqtt_server = "192.168.3.32";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// TODO : REPLACE WITH YOUR DESTINATION'S ESP MAC ADDRESS (for ESP NOW)
// WARNING : First index of moteAddress matrix must correspond to moteBoardId (0, 1, etc.)
// If there are more than one mote, use the following syntax (example with MOTE_COUNT=2) :
// uint8_t moteAddress[MOTE_COUNT][6] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
//                              {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
uint8_t moteAddress[MOTE_COUNT][6] = {0xEC, 0x62, 0x60, 0x10, 0xA7, 0x84};
esp_now_peer_info_t peerInfo[MOTE_COUNT];

// ================================================================================================
// DATA STRUCTURES DEFINITION FOR ESPNOW COMMUNICATION
// (Can be modified according to your needs, warning: must be identical on mote and sink)
//
// Create structure to exchange data through ESP-NOW : Mote->Sink
typedef struct struct_mote2sinkMessage {
  int boardId;
  int readingId;
  int timeTag;
  float data0;
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  bool bool0;
  bool bool1;
  bool bool2;
  bool bool3;
  char text[64];
} struct_mote2sinkMessage;
struct_mote2sinkMessage espNow_incomingReadings;
// Create an array with all the last incoming messages, sorted by board ID
struct_mote2sinkMessage espNow_lastMotesReadings[MOTE_COUNT] = {};

// Create structure to exchange data through ESP-NOW : Sink->Mote
typedef struct struct_sink2moteMessage {
  int boardId;
  float data0;
  float data1;
  bool bool0;
  bool bool1;
  bool bool2;
  bool bool3;
  char text[64];
} struct_sink2moteMessage;
struct_sink2moteMessage espNow_outgoingMessage;

// ================================================================================================
// Functions for ESP NOW communications
//

// callback function that will be executed when data is received (mote->sink)
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from MAC = ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(", length = ");
  Serial.print(len);
  Serial.println(" Bytes");
  memcpy(&espNow_incomingReadings, incomingData, sizeof(espNow_incomingReadings));

  // Make a local copy of last received board data
  espNow_lastMotesReadings[espNow_incomingReadings.boardId] = espNow_incomingReadings;
  Serial.print(" - boardID : ");
  Serial.println(espNow_incomingReadings.boardId);
  Serial.print(" - readingID : ");
  Serial.println(espNow_incomingReadings.readingId);
  Serial.print(" - timeTag : ");
  Serial.println(espNow_incomingReadings.timeTag);
  Serial.print(" - data0 : ");
  Serial.println(espNow_incomingReadings.data0);
  Serial.print(" - data1  : ");
  Serial.println(espNow_incomingReadings.data1);
  Serial.print(" - data2  : ");
  Serial.println(espNow_incomingReadings.data2);
  Serial.print(" - data3  : ");
  Serial.println(espNow_incomingReadings.data3);
  Serial.print(" - data4  : ");
  Serial.println(espNow_incomingReadings.data4);
  Serial.print(" - data5  : ");
  Serial.println(espNow_incomingReadings.data5);
  Serial.print(" - bool0 : ");
  Serial.println(espNow_incomingReadings.bool0);
  Serial.print(" - bool1 : ");
  Serial.println(espNow_incomingReadings.bool1);
  Serial.print(" - bool2 : ");
  Serial.println(espNow_incomingReadings.bool2);
  Serial.print(" - bool3 : ");
  Serial.println(espNow_incomingReadings.bool3);
}

// callback when data is sent from sink (sink->mote)
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void espNowSendDataToMote(struct_sink2moteMessage message) {
  esp_err_t result = esp_now_send(moteAddress[espNow_outgoingMessage.boardId],
                                  (uint8_t *)&message, sizeof(struct_sink2moteMessage));

  if (result == ESP_OK) {
    Serial.print("Sent with success to MAC address : ");
    char macStr[18];
    uint8_t *mac_addr = moteAddress[espNow_outgoingMessage.boardId];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println(macStr);
  } else {
    Serial.println("Error sending the data");
  }
}

// ================================================================================================
// Functions to handle MQTT subscribed topic update
// TODO : to modify according to project needs, code below is just an example
void mqttCallback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/board0/output, you check if the message is either "on" or "off".
  // Send a message through ESP NOW according to the MQTT message
  espNow_outgoingMessage = {};
  if (String(topic) == "esp32/board0/output") {
    espNow_outgoingMessage.boardId = 0;
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("Board0 led on");
      espNow_outgoingMessage.bool0 = 1;
      // digitalWrite(ledPin, HIGH);
      // TODO : send message to ESP NOW to switch ON LED
    } else if (messageTemp == "off") {
      Serial.println("Board0 led off");
      espNow_outgoingMessage.bool0 = 0;
      // digitalWrite(ledPin, LOW);
      // TODO : send message to ESP NOW to switch ON LED
    }
    char textMsg[] = "esp32/board0/output has been updated";
    memcpy(&espNow_outgoingMessage.text, textMsg, sizeof(textMsg));
    espNowSendDataToMote(espNow_outgoingMessage);
  }
  // Same for board1
  if (String(topic) == "esp32/board1/output") {
    espNow_outgoingMessage.boardId = 1;
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("Board1 led on");
      espNow_outgoingMessage.bool0 = 1;
      // digitalWrite(ledPin, HIGH);
      // TODO : send message to ESP NOW to switch ON LED
    } else if (messageTemp == "off") {
      Serial.println("Board1 led off");
      espNow_outgoingMessage.bool0 = 0;
      // digitalWrite(ledPin, LOW);
      // TODO : send message to ESP NOW to switch ON LED
    }
    char textMsg[] = "esp32/board1/output has been updated";
    memcpy(&espNow_outgoingMessage.text, textMsg, sizeof(textMsg));
    espNowSendDataToMote(espNow_outgoingMessage);
  }
}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/board0/output");
      client.subscribe("esp32/board1/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// ================================================================================================
// Setup and Loop functions below
//
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // TODO : Here perform setup of hardware which is connected to sink (RFID, dht, LEDs, etc.), if needed
  // e.g. :
  //  - pinMode(ledPin1, OUTPUT);
  //  - pinMode(ledPin2, OUTPUT);

  //-----------------------------------------------------------
  // Settings for Wifi configuration
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("Sink MAC Address: ");
  Serial.println(WiFi.macAddress());

  //-----------------------------------------------------------
  // Settings for ESP NOW configuration
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESP-NOW is successfully initialized, we will register for recv CB to
  // get recv packet info, and we will register to send info
  esp_now_register_recv_cb(espNowOnDataRecv);
  esp_now_register_send_cb(espNowOnDataSent);

  // Register peers (motes)
  for (int i = 0; i < MOTE_COUNT; i++) {
    peerInfo[i].channel = 0;
    peerInfo[i].encrypt = false;
    memcpy(peerInfo[i].peer_addr, moteAddress[i], 6);
    if (esp_now_add_peer(&peerInfo[i]) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  //-----------------------------------------------------------
  // Settings for MQTT configuration
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

// TODO : Modify code below according to project needs, the following is
// only an example (MQTT publishing)
void loop() {
  // MQTT reconnect when needed
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;
    for (size_t i = 0; i < MOTE_COUNT; i++) {
      if (espNow_lastMotesReadings[i].timeTag != 0) {
        char lastUpdateString[8];
        char tempString[8];
        char humString[8];
        // Switch case on "BoardId"
        switch (espNow_lastMotesReadings[i].boardId) {
          case 0:
            dtostrf(espNow_lastMotesReadings[i].timeTag, 1, 2, lastUpdateString);
            client.publish("esp32/board0/lastUpdate", lastUpdateString);

            dtostrf(espNow_lastMotesReadings[i].data0, 1, 2, tempString);
            client.publish("esp32/board0/temperature", tempString);

            dtostrf(espNow_lastMotesReadings[i].data1, 1, 2, humString);
            client.publish("esp32/board0/humidity", humString);

            espNow_lastMotesReadings[i] = {};
            break;

          case 1:
            dtostrf(espNow_lastMotesReadings[i].timeTag, 1, 2, lastUpdateString);
            client.publish("esp32/board1/lastUpdate", lastUpdateString);

            dtostrf(espNow_lastMotesReadings[i].data0, 1, 2, tempString);
            client.publish("esp32/board1/temperature", tempString);

            dtostrf(espNow_lastMotesReadings[i].data1, 1, 2, humString);
            client.publish("esp32/board1/humidity", humString);

            espNow_lastMotesReadings[i] = {};
            break;

          default:
            break;
        }
      }
    }
  }
}
