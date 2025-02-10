// #include <Arduino.h>
// #define PIN 36      //Read Pin

// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   // read the value from the sensor:
//   Serial.println(analogRead(PIN));
//   delay(10);
// }

#include <Arduino.h>
#define RESOLUTION 12   //ADC resoltuion
#define MAXVALUE pow(2,RESOLUTION)  //Resolution corresponding value 
#define PRECISION 10   //Admissible error 
#define PIN 36    //Read pin 

uint32_t ADKeyVal[10] = {0};   //Corresponding key value 
uint32_t ADCKeyIn = 0;

void ADKeybegin(){
  float RESValue[10] = {0, 3, 6.2, 9.1, 15, 24, 33, 51, 100, 220};  //Resistor resistance 
  for(uint8_t i = 0; i < 10; i++){
    ADKeyVal[i] = RESValue[i]/(RESValue[i]+22)*MAXVALUE;
  }
}

void setup() {
  Serial.begin(115200);
  ADKeybegin();
}

int8_t getKey(){
  for(uint8_t i = 0; i < 10;i++){
    if(ADCKeyIn > ADKeyVal[i]){
      if((ADCKeyIn - ADKeyVal[i]) < PRECISION){
        return i;
      }
    } else{
      if((ADKeyVal[i] - ADCKeyIn) < PRECISION){
        return i;
      }
    }
  }
  return -1;
}

void loop() {
  ADCKeyIn = analogRead(PIN);
  if(ADCKeyIn < (MAXVALUE-PRECISION)){
    Serial.print("K = ");
    Serial.println(getKey());
    Serial.println(ADCKeyIn);
  }
  delay(10);
}


