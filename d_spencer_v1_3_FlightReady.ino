/***********************************
 * CANSAT COMPETITION 2024
 * Team: D-spencer [dspencer.cansat.team@gmail.com]
 * D-spencer SW Version: v1.3 (Flight Ready)
 * Team Members:
 *      António Cruz
 *      Constança Vasco
 *      Duarte Carvalho
 *      Joana Melo
 *      João Saltão
 *      Matilde Mendes
 *
 * Used Hardware in Project:
 *      Arduino Nano
 *      Adafruit BMP280 - Temperature, Pressure
 *      NEO7M - GPS
 *      SG90 - Servo
 *      APC 220 - Antenna
 * Date: Jan..May 2024
************************************/

//------------------------------------------------------
// Libraries used
//------------------------------------------------------
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Servo.h> 
#include <TinyGPSPlus.h> 

//------------------------------------------------------
// Constants and global variables
//------------------------------------------------------
#ifndef pinRX
#define pinRX 6
#endif

#ifndef pinTX
#define pinTX 7
#endif

#define ServoPin 9
#define GPSpinRX 3
#define GPSpinTX 4
#define GPSBaud 9600

const float z = 0.0065, R = 287.06, g = 9.81;
float T0 = 0; // Base Temperature at start
float P0 = 0; // Base Pressure at start

unsigned long sendDatapreviousTime = 0;
#define INTERVAL_MS_SEND_DATA 1000

unsigned long velocityCalcpreviousTime = 0; 
#define INTERVAL_MS_CALC_VELOCITY 1000

#define UNINITIALIZED_VELOCITY -99999.0  
float previousAltitude = UNINITIALIZED_VELOCITY; 
float velocity = 0; 

const float DESCENT_RELEVANT_VELOCITY = -1.0;
const float ALTITUDE_M_TO_OPEN_DOOR = 100; 

const unsigned long intervalMsInRelevantVelocity = 2000; 
const unsigned long timeInFutureConst = (unsigned long) -1; 
unsigned long inDescentRelevantVelocityPreviousTime = timeInFutureConst; 

enum VelocityStageEnum {
  Starting = 0,
  InFlight = 1,
  RelevantDescent = 2,
  TestingAltitudeAndOpenDoor = 3,
  AlreadyOpenedDoor = 4,
  SendDataWithGPSInfo = 5
};
VelocityStageEnum velocityStage = Starting;

int _loop = 0;

TinyGPSPlus gps;
SoftwareSerial ss(GPSpinRX, GPSpinTX);
bool isGpsInit = false;

Adafruit_BMP280 bmp280; 
SoftwareSerial apc220(pinRX, pinTX); 
Servo servo;

double latLastValid = 0.0;
double lngLastValid = 0.0;

//------------------------------------------------------
// Setup
//------------------------------------------------------
void setup() {
  unsigned long nowMillis = millis();
  Serial.begin(9600);
  delay(200);

  Serial.print("t:");
  Serial.print(nowMillis);
  Serial.println("; setup() strt");
  apc220.begin(9600);
  delay(200);
  
  bmp280.begin();

  servo.attach(ServoPin);
  servo.write(0);
  servo.detach(); 

  delay(200);
  Serial.println("; setup() exit");
}

//------------------------------------------------------
// Main Loop
//------------------------------------------------------
void loop() {
  bool isGpsOk = false;

  if (_loop < 1) {
    delay(300);
    _loop = 1;
    return;
  }

  if(_loop == 1) {
    P0 = bmp280.readPressure()/100;
    T0 = bmp280.readTemperature();
    _loop = 2;
    return;
  }

  if (isGpsInit) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        double lat = 0.0, lng = 0.0;
        isGpsOk = GpsInfo(&lat, &lng);
        if(isGpsOk){
          latLastValid = lat;  
          lngLastValid = lng;
        }
      }
    }
  }

  unsigned long actualTime = millis();

  // Read real sensor values
  float T = bmp280.readTemperature();
  float P = bmp280.readPressure() / 100;
  float H = ((T0 + 273.15) / z) * (1 - (pow(P / P0, (z * R) / g)));

  if(previousAltitude == UNINITIALIZED_VELOCITY){
    previousAltitude = H;
  }

  if (actualTime - velocityCalcpreviousTime >= INTERVAL_MS_CALC_VELOCITY) {
    velocityCalcpreviousTime = actualTime;
    velocity = (H - previousAltitude) / ((float) INTERVAL_MS_CALC_VELOCITY / 1000.0);
    previousAltitude = H;
  }

  switch(velocityStage){
    case Starting:
      Serial.println(F("Initialize GPS"));
      ss.begin(GPSBaud);
      isGpsInit = true;
      ShowStageChanging(actualTime, velocityStage, InFlight, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
      velocityStage = InFlight;
      break;

    case InFlight:
      if(velocity < DESCENT_RELEVANT_VELOCITY) {
        ShowStageChanging(actualTime, velocityStage, RelevantDescent, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = RelevantDescent;
        inDescentRelevantVelocityPreviousTime = actualTime;
      }
      break;

    case RelevantDescent:
      if(velocity < DESCENT_RELEVANT_VELOCITY) {
        if(actualTime - inDescentRelevantVelocityPreviousTime > intervalMsInRelevantVelocity) {
          ShowStageChanging(actualTime, velocityStage, TestingAltitudeAndOpenDoor, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
          velocityStage = TestingAltitudeAndOpenDoor; 
        }
      } else {
        ShowStageChanging(actualTime, velocityStage, InFlight, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = InFlight;
      }
      break;

    case TestingAltitudeAndOpenDoor:
      if(H < ALTITUDE_M_TO_OPEN_DOOR){
        servo.attach(ServoPin);
        delay(100);
        servo.write(90);
        delay(300);
        servo.detach();
        ShowStageChanging(actualTime, velocityStage, AlreadyOpenedDoor, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = AlreadyOpenedDoor;
      }
      break;

    case AlreadyOpenedDoor:
      ss.begin(GPSBaud);
      ShowStageChanging(actualTime, velocityStage, SendDataWithGPSInfo, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
      velocityStage = SendDataWithGPSInfo;
      break;

    case SendDataWithGPSInfo:
      break; 
  }

  // Data Transmission
  if (actualTime - sendDatapreviousTime >= INTERVAL_MS_SEND_DATA) {
    sendDatapreviousTime = actualTime;
    char sT_Lat[12], sP_Lng[12], sH[12], sBufferSprintf[61];

    dtostrf(T, -8, 2, sT_Lat);
    dtostrf(P, -8, 2, sP_Lng);
    dtostrf(H, -8, 2, sH);

    sprintf(sBufferSprintf, "t:%lu ; T:%s ; P:%s ; H:%s", millis(), sT_Lat, sP_Lng, sH);

    Serial.print(sBufferSprintf);
    apc220.print(sBufferSprintf);

    if (isGpsInit) {
      dtostrf(latLastValid, 11, 6, sT_Lat);
      dtostrf(lngLastValid, 11, 6, sP_Lng);
      sprintf(sBufferSprintf, " ; Lt:%s ; Lg:%s", sT_Lat, sP_Lng);
      Serial.print(sBufferSprintf);
      apc220.print(sBufferSprintf);
    }
    Serial.println();
    apc220.println();
  }
}

void ShowStageChanging(unsigned long actualTime, VelocityStageEnum currentVelocityStage, VelocityStageEnum newVelocityStage, 
                       float velocity, float descentRelevantVelocity, 
                       unsigned long inDescentRelevantVelocityPreviousTime, unsigned long intervalMsInRelevantVelocity){
  char sBufferSprintf[250], sVelocity[11], sDescentRelevantVelocity[11];
  dtostrf(velocity, -8, 2, sVelocity);
  dtostrf(descentRelevantVelocity, -8, 2, sDescentRelevantVelocity);

  sprintf(sBufferSprintf, "CHANGING STAGE! t:%lu ; CurrStage:%d ; NewStage:%d ; Vel:%s ; relVel:%s", 
          millis(), currentVelocityStage, newVelocityStage, sVelocity, sDescentRelevantVelocity);

  Serial.println(sBufferSprintf);
  apc220.println(sBufferSprintf);
}

bool GpsInfo(double *lat, double *lng) {
  if (gps.location.isValid()) {
    *lat = gps.location.lat();
    *lng = gps.location.lng();
    return true;
  }
  return false;
}