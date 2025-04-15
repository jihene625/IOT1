#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Tomoto_HM330X.h>
#include <Arduino.h>


#define BOARD_ID 2


Tomoto_HM330X sensor;


// Wifi Access Point SSID
const char *ssid = "iot";


unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 2000;       // Interval at which to publish sensor readings
unsigned int readingId = 0;


// TODO : change variables below according to project needs
const int ledPin = 5;
const int potPin = 36; // potentiomètre
float temperature = 0;
float humidity = 0;
const int motorControl = 25;


// MAC Address of the sink
// TODO : enter your sink MAC address here
uint8_t broadcastAddress[] = {0xEC, 0x62, 0x60, 0x5A, 0x7E, 0x78};
esp_now_peer_info_t peerInfo;


// ================================================================================================
// DATA STRUCTURES DEFINITION FOR ESPNOW COMMUNICATION
// (Can be modified according to your needs, warning : must be identical on mote and sink)
//
// Create structure to exchange data through ESP-NOW : Mote->Sink
typedef struct struct_mote2sinkMessage
{
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
// Create a struct_message
struct_mote2sinkMessage espNow_moteData;


// Create structure to exchange data through ESP-NOW : Sink->Mote
typedef struct struct_sink2moteMessage
{
  int boardId;
  float data0;
  float data1;
  bool bool0;
  bool bool1;
  bool bool2;
  bool bool3;
  char text[64];
} struct_sink2moteMessage;
struct_sink2moteMessage espNow_incomingMessage;


// ================================================================================================
// Sensor readings :
// TODO : the functions below are examples, adapt to project needs


float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float readPM25()
{
  // Essai de lecture du capteur ; sensor.readSensor() retourne false en cas d'échec
  if (!sensor.readSensor())
  {
    Serial.println("Échec de la lecture du capteur HM330X!");
    return 0;
  }
  else
  {
    // Retourne la valeur de concentration en PM2.5 selon la norme CF=1
    return sensor.std.getPM2_5();
  }
}


// ================================================================================================
// Functions for ESP NOW communications
//
// Callback function that will be executed when data is received (sink -> mote)
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from MAC = ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(", length = ");
  Serial.print(len);
  Serial.println(" Bytes");
  memcpy(&espNow_incomingMessage, incomingData, sizeof(espNow_incomingMessage));
  Serial.print(" - boardID : ");
  Serial.println(espNow_incomingMessage.boardId);
  Serial.print(" - data0 : ");
  Serial.println(espNow_incomingMessage.data0);
  Serial.print(" - data1  : ");
  Serial.println(espNow_incomingMessage.data1);
  Serial.print(" - bool0 : ");
  Serial.println(espNow_incomingMessage.bool0);
  Serial.print(" - bool1 : ");
  Serial.println(espNow_incomingMessage.bool1);
  Serial.print(" - bool2 : ");
  Serial.println(espNow_incomingMessage.bool2);
  Serial.print(" - bool3 : ");
  Serial.println(espNow_incomingMessage.bool3);


  // Make a local copy of last received board data
  if (espNow_incomingMessage.boardId != BOARD_ID)
  {
    Serial.println("Message does not correspond to this board Id");
    return;
  }


  Serial.println("Est-ce que ça marche ???????");
  Serial.println(espNow_incomingMessage.bool0);


  if (espNow_incomingMessage.bool0 == 1)
  {
    digitalWrite(motorControl, HIGH);
    Serial.println("Motor ON");
  }
  else if (espNow_incomingMessage.bool0 == 0)
  {
    digitalWrite(motorControl, LOW);
    Serial.println("Motor OFF");
  }
}


// Callback when data is sent  (mote -> sink)
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks())
  {
    for (uint8_t i = 0; i < n; i++)
    {
      if (!strcmp(ssid, WiFi.SSID(i).c_str()))
      {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}


// ================================================================================================
// Setup and Loop functions below
//
void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);
  delay(100);


  // make the transistor's pin an output:


  pinMode(motorControl, OUTPUT);
  digitalWrite(motorControl, LOW);
  Serial.println("Motor OFF");


  // Initialisation du capteur
  if (!sensor.begin())
  {
    Serial.println("Échec d'initialisation du HM330X");
    while (1)
      ; // Boucle infinie en cas d'erreur
  }


  Serial.println("HM330X initialisé");
  Serial.println();


  //-----------------------------------------------------------
  // Settings for Wifi configuration


  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  int32_t channel = getWiFiChannel(ssid);
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after


  //-----------------------------------------------------------
  // Settings for ESP NOW configuration
  //
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet and get recv packer info
  esp_now_register_send_cb(espNowOnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(espNowOnDataRecv));


  // Register peer (sink)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;


  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}


// TODO : Modify code below according to project needs, the following is
// only an example (simulation of temperature and humidity)
void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
#if BOARD_ID == 2
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    espNow_moteData = {};


    // digitalWrite(motorControl, HIGH);
    // Serial.println("Motor ON");
    // delay(2000);


    // Set values to send
    espNow_moteData.boardId = BOARD_ID;
    espNow_moteData.readingId = readingId++;
    espNow_moteData.timeTag = currentMillis;
    espNow_moteData.data2 = readPM25();
    espNow_moteData.readingId = readingId++;
    char textMsg[] = "Hi Sink, here's my data for you: ";
    memcpy(&espNow_moteData.text, textMsg, sizeof(textMsg));


    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espNow_moteData, sizeof(espNow_moteData));


    Serial.println("particules");
    Serial.println(espNow_moteData.data2);




    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
#endif
  }
}

