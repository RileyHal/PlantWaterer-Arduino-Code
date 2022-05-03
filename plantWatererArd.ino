#include<DHT.h>
#include <ArduinoJson.h>
#define DHTTYPE DHT11
#define DHTPIN 48

//Digital Pins
int waterDetectionPinIn = 50;
int relayPinOut = 52; //For the water pump later on
DHT dht(DHTPIN, DHTTYPE);

//Analog Pins
int lightSensorPinIn = A0;
int soilMoisturePinIn = A1;
int temperaturePinIn = A2;

//Default Values
int loopDelay = 1000;
int mode = 0;
long duration=1; 
long interval = 65000;
//completely dry is 1024, wet is lower;
int targetMoisture = 1024;


float temperature, humidity;
int lightVal, moistureVal, isWater;
boolean pumpActive;
unsigned long startMillis, currentMillis, difference;

void setup() {
  // put your setup code here, to run once:
  pinMode(relayPinOut, OUTPUT);
  pinMode(waterDetectionPinIn, INPUT);
  dht.begin();
  startMillis = millis();

  Serial.begin(9600);
}

void sendData(){
  lightVal = analogRead(lightSensorPinIn);
  moistureVal = analogRead(soilMoisturePinIn);
  isWater = digitalRead(waterDetectionPinIn);
  temperature = (dht.readTemperature()*1.8) + 32;
  humidity = dht.readHumidity();
  String msg = "{\"Light\":" + String(lightVal) + ",\"Moisture\":" + String(moistureVal) + ",\"soilMoistureTarget\":" + String(targetMoisture) + ",\"isWater\":" + String(isWater) + ",\"pumpActive\":" + String(pumpActive)
  + ",\"temperature\":" + String(temperature) + ",\"Humidity\":" + String(humidity)+"}";
  Serial.println(msg);
  //end send data
}

void toggleMotorTimer(){
  digitalWrite(relayPinOut,HIGH);
  delay(duration * 1000);
  digitalWrite(relayPinOut,LOW);
}

void checkSerial(){
  if(Serial.available()){
      String receivedDataRaw = Serial.readString();
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, receivedDataRaw);
      // Test if parsing succeeds.
      if (error) {
        return;
      }
      //set variables
      mode = doc["mode"];
      duration = doc["duration"];
      interval = doc["interval"];
      targetMoisture = doc["soilMoistureTarget"];
  }
}

void autoMode(){
  //if moisture val is greater than the target(soil moisture is lower than the target), run pumps
  if(moistureVal > targetMoisture && isWater == 1){
      digitalWrite(relayPinOut,HIGH);
      //turn on for 1.25 seconds
      delay(1.25 * 1000);
      digitalWrite(relayPinOut,LOW);
      //delay for 5 seconds to let the soil moisture sensor detect change
      delay(10000);
  }
}

void timerMode(){
  currentMillis = millis();
  difference = currentMillis - startMillis;
  if (difference >= (interval*1000) && isWater == 1){
    //do stuff here
    toggleMotorTimer();
    startMillis = millis();
  }
}

void doFunction(){
  //mode 0 not included because it is the off state
  if (mode == 1){
    autoMode();
  } else if(mode == 2) {
    timerMode();
  }
}

void loop() {
  delay(loopDelay);
  // put your main code here, to run repeatedly:
  sendData();
  checkSerial();
  doFunction();
}
