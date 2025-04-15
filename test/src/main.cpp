#include <fallDetection_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Définition
#define BOARD_ID 3
#define FREQUENCY_HZ 60
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))

// MPU
Adafruit_MPU6050 mpu;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;
unsigned long last_interval_ms = 0;

// ESP-NOW
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x5A, 0x7E, 0x78};
esp_now_peer_info_t peerInfo;

// Structure d'envoi
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
  char text2[64];
} struct_mote2sinkMessage;

struct_mote2sinkMessage espNow_moteData;
unsigned int readingId = 0;

// Callback d'envoi
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Canal WiFi
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = {0};
  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  if (r > 0) Serial.write(print_buf);
}

void setup() {
  Serial.begin(115200);

  // I2C pour MPU
  Wire.begin(4, 5);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // WiFi
  WiFi.mode(WIFI_STA);

  Serial.print("Adresse MAC : ");
  Serial.println(WiFi.macAddress());

  int32_t channel = getWiFiChannel("iot");
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(espNowOnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Setup complete.");
}

void loop() {
  sensors_event_t a, g, temp;

  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    mpu.getEvent(&a, &g, &temp);

    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;

    if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      signal_t signal;
      ei_impulse_result_t result;

      int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
      }

      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
      if (res != EI_IMPULSE_OK) return;

      float max_value = 0.0;
      size_t max_index = 0;
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > max_value) {
          max_value = result.classification[ix].value;
          max_index = ix;
        }
      }

      // Affichage
      const char* predicted_label = result.classification[max_index].label;
      Serial.print("Predicted class: ");
      Serial.println(predicted_label);

      // Préparer message
      espNow_moteData = {};
      espNow_moteData.boardId = BOARD_ID;
      espNow_moteData.readingId = readingId++;
      espNow_moteData.timeTag = millis();
      strncpy(espNow_moteData.text2, predicted_label, sizeof(espNow_moteData.text2));

      // Envoi ESP-NOW
      esp_err_t result_send = esp_now_send(broadcastAddress, (uint8_t *)&espNow_moteData, sizeof(espNow_moteData));
      if (result_send == ESP_OK) {
        Serial.println("Classe envoyée avec succès");
      } else {
        Serial.println("Erreur d'envoi de la classe");
      }

      feature_ix = 0;
    }
  }
}
