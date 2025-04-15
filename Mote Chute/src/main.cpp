#include <fallDetection_inferencing.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQUENCY_HZ        60
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

#define led1 2
#define led2 14
#define led3 12

Adafruit_MPU6050 mpu;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

static unsigned long last_interval_ms = 0;

void setup() {
  Serial.begin(115200);

  // Initialisation des LEDs
  pinMode(led1, OUTPUT); digitalWrite(led1, LOW);
  pinMode(led2, OUTPUT); digitalWrite(led2, LOW);
  pinMode(led3, OUTPUT); digitalWrite(led3, LOW);

  // Initialisation I2C avec broches personnalisées pour ESP32
  Wire.begin(4, 5); // SDA = IO4, SCL = IO5

  // Initialisation MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
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

      // Détermination de la classe avec la plus haute probabilité
      float max_value = 0.0;
      size_t max_index = 0;

      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > max_value) {
          max_value = result.classification[ix].value;
          max_index = ix;
        }
      }

      // Affiche uniquement la classe détectée
      Serial.println(result.classification[max_index].label);

      // LEDs selon la classe détectée
      digitalWrite(led1, max_index == 0 ? HIGH : LOW);
      digitalWrite(led2, max_index == 1 ? HIGH : LOW);
      digitalWrite(led3, max_index >= 2 ? HIGH : LOW);

      feature_ix = 0;
    }
  }
}

void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}