/***********************************
 * CANSAT COMPETITION 2024
 * Team: D-spencer [dspencer.cansat.team@gmail.com]
 * D-spencer SW Version: v1.3
 * Team Members:
 *     António Cruz
 *     Constança Vasco
 *     Duarte Carvalho
 *     Joana Melo
 *     João Saltão
 *     Matilde Mendes
 *
 * Used Hardware in Project:
 *     Arduino Nano
 *     Adafruit BMP280 - Temperature, Pressure
 *     NEO7M - GPS
 *     SG90 - Servo
 *     APC 220 - Antenna
 * Date: Jan..May 2024
************************************/


//------------------------------------------------------
// Libraries used
//------------------------------------------------------
//#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Servo.h> // Incluir a biblioteca servo

#include <TinyGPSPlus.h> // Include GPS library

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


// in test mode, these are the pressures that will be simulated

int _inTestMode = true;
int _testModePressureIdx = 0; // in test mode will keep the index to get the pressure values at each time interval
// time interval (in ms) to read pressure from test values:
#define TEST_MODE_INTERVAL_MS_READ_PRESSURE 500
unsigned long _testModeIdxPreviousTime = 0; // last time we increased idx to get next test values from pressure values in test mode
#define TEST_MODE_TOT_VALUES 86

// pressure test values in hPa:
float _testModePressureHPaValues[TEST_MODE_TOT_VALUES] = {
  1000.000000,  // Alt = 0.00
  996.373664,  // Alt = 30.10
  992.772460,  // Alt = 60.10
  989.184272,  // Alt = 90.10
  985.609052,  // Alt = 120.10
  982.046755,  // Alt = 150.10
  978.497333,  // Alt = 180.10
  974.960740,  // Alt = 210.10
  971.436929,  // Alt = 240.10
  967.925854,  // Alt = 270.10
  964.427469,  // Alt = 300.10
  960.941729,  // Alt = 330.10
  957.468587,  // Alt = 360.10
  954.007998,  // Alt = 390.10
  950.559917,  // Alt = 420.10
  947.124298,  // Alt = 450.10
  943.701097,  // Alt = 480.10
  940.290268,  // Alt = 510.10
  936.891766,  // Alt = 540.10
  940.176786,  // Alt = 511.10
  940.063318,  // Alt = 512.10
  940.108703,  // Alt = 511.70
  940.120050,  // Alt = 511.60
  940.188133,  // Alt = 511.00
  940.176786,  // Alt = 511.10
  940.176786,  // Alt = 511.10
  940.290268,  // Alt = 510.10
  940.403763,  // Alt = 509.10
  940.517272,  // Alt = 508.10
  940.630795,  // Alt = 507.10
  940.744332,  // Alt = 506.10
  940.630795,  // Alt = 507.10
  940.630795,  // Alt = 507.10
  942.562781,  // Alt = 490.10
  944.840787,  // Alt = 470.10
  946.552903,  // Alt = 455.10
  948.268122,  // Alt = 440.10
  949.413328,  // Alt = 430.10
  950.559917,  // Alt = 420.10
  951.707890,  // Alt = 410.10
  952.857250,  // Alt = 400.10
  954.007998,  // Alt = 390.10
  955.160136,  // Alt = 380.10
  956.313665,  // Alt = 370.10
  957.468587,  // Alt = 360.10
  958.624904,  // Alt = 350.10
  959.782617,  // Alt = 340.10
  960.941729,  // Alt = 330.10
  962.102240,  // Alt = 320.10
  963.264153,  // Alt = 310.10
  964.427469,  // Alt = 300.10
  965.592190,  // Alt = 290.10
  966.758318,  // Alt = 280.10
  967.925854,  // Alt = 270.10
  969.094800,  // Alt = 260.10
  970.265158,  // Alt = 250.10
  971.436929,  // Alt = 240.10
  972.610115,  // Alt = 230.10
  973.784718,  // Alt = 220.10
  974.960740,  // Alt = 210.10
  976.138182,  // Alt = 200.10
  977.317045,  // Alt = 190.10
  978.497333,  // Alt = 180.10
  979.679046,  // Alt = 170.10
  980.862186,  // Alt = 160.10
  982.046755,  // Alt = 150.10
  983.232755,  // Alt = 140.10
  984.420186,  // Alt = 130.10
  985.609052,  // Alt = 120.10
  986.799354,  // Alt = 110.10
  987.991093,  // Alt = 100.10
  989.184272,  // Alt = 90.10
  990.378891,  // Alt = 80.10
  991.574953,  // Alt = 70.10
  992.772460,  // Alt = 60.10
  993.971412,  // Alt = 50.10
  995.171813,  // Alt = 40.10
  996.373664,  // Alt = 30.10
  997.576965,  // Alt = 20.10
  998.781721,  // Alt = 10.10
  998.781721,  // Alt = 10.10
  998.781721,  // Alt = 10.10
  998.781721,  // Alt = 10.10
  998.781721,  // Alt = 10.10
  998.781721,  // Alt = 10.10
  998.781721  // Alt = 10.10
};


const float z = 0.0065, R = 287.06, g = 9.81;
float T0 = 0; // This will hold (base) Temperature at the time when arduino/program started. Will be assigned later
float P0 = 0; // This will hold (base) Pressure at the time when arduino/program started. Will be assigned later

unsigned long sendDatapreviousTime = 0;

// send data to apc220 antena and USB Serial at each INTERVAL_MS_SEND_DATA milliseconds.
// according to mission 1, it must be once per second minimum (1000 ms):
#define INTERVAL_MS_SEND_DATA 1000

unsigned long velocityCalcpreviousTime = 0;  // last time it calculated velocity
// calculate vertical velocity at each INTERVAL_MS_CALC_VELOCITY milliseconds. Altitude is based on pressure, and sensor is not too acurate...:
#define INTERVAL_MS_CALC_VELOCITY 1000

// just a "marker", a "big negative" number to know when that altitude was not initialized yet, so velocity will take the same vertical point (velocity 0):
#define UNINITIALIZED_VELOCITY -99999.0  

float previousAltitude = UNINITIALIZED_VELOCITY; // needed to calc velocity, i.e., only with 2 valid measures, can we calculate it
float velocity = 0;  // vertical velocity, (delta altitude) / (delta t)

// we call DESCENT_RELEVANT_VELOCITY one that is negative (i.e., descending), but it can be slow. We just want to compare the real velocity with this one, to make sure it is not a "sensor glitch"
const float DESCENT_RELEVANT_VELOCITY = -1.0;
const float ALTITUDE_M_TO_OPEN_DOOR = 100; // will open door below this altitude (in meters)

// TODO: check the best value: 1000ms?, 2000ms? 3000ms? 5000ms? 7000ms? other?
const unsigned long intervalMsInRelevantVelocity = 2000; // only after beeing this time (in ms) "below the (absolute) relevant" velocity, it will open the door
const unsigned long timeInFutureConst = (unsigned long) -1; // ULONG_MAX;  // -1 converted to an unsigned should give the maximum positive number allowed in unsigned long variables 
unsigned long inDescentRelevantVelocityPreviousTime = timeInFutureConst;   // last time it entered the relevant velocity (base for door opening). Just assume it is in future, meaning no time in this stage

enum VelocityStageEnum {  // "state machine"
  Starting = 0,                          // system starting up
  InFlight = 1,                          // program started and object in ready to be launched or is already in normal flight
  RelevantDescent = 2,                   // object is descending
  TestingAltitudeAndOpenDoor = 3,        // we are testing if altitude is below the right one to open door. Open door when the altitude is right.
  AlreadyOpenedDoor = 4,                 // we did open the door (nothing much more to be done)
  SendDataWithGPSInfo = 5                // after the door was opened, most important is to send gps info to antenna. The object will shortly be stationay somewhere on the ground
};
VelocityStageEnum velocityStage = Starting;   // program is starting

int _loop = 0;


// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(GPSpinRX, GPSpinTX);
bool isGpsInit = false;   // false if not initialized; true if initialized

Adafruit_BMP280 bmp280; 

SoftwareSerial apc220(pinRX, pinTX); 

Servo servo;


double latLastValid = 0.0;  // last known valid latitude; 0.0 means no valid yet
double lngLastValid = 0.0;  // last known valid longitude; 0.0 means no valid yet


//------------------------------------------------------
// Setup function. Note that some further HW set up is done
// in the loop( ) itself, for example the servo code
//------------------------------------------------------
void setup() {

  unsigned long nowMillis = millis( );
  // setup of: communication, sensor, and emission module 
  Serial.begin(9600);
  delay(200);

  Serial.print("t:");
  Serial.print(nowMillis);
  Serial.println("; setup() strt");
  apc220.begin(9600);
  delay(200);
  apc220.print("t:");
  apc220.print(nowMillis);
  apc220.println("; setup() strt");
 
  bmp280.begin();

  // servo stuff
  servo.attach(ServoPin);  // attaching the servo object created to pin ServoPin
  servo.write(0);  // servo position to initial position
  servo.detach();  // avoid interferences with antenna / gpps; we just need to open later stage

  delay(200);

  Serial.print("t:");
  Serial.print(millis());
  Serial.println("; setup() exit");

  apc220.print("t:");
  apc220.print(millis());
  apc220.println("; setup() exit");
}


//------------------------------------------------------
// Main loop function, executes "in loop" called by the system again and again
//------------------------------------------------------
void loop() {
  bool useGpsHw = true;
  bool isGpsOk = false;         // will be overiden below, either by hw read below or hard coded below

  // in first run (_loop == 0) wait a little bit to "let hw stabilize"
  if (_loop < 1) {
    delay(300); // wait a little bit to "let hw stabilize"?
    _loop = 1;  // global var _loop passes to 1
    return;
  }


  // on second iteration (_loop == 1), pressure and temperature values are stored in P0 and P1
  if(_loop == 1) {
    if(_inTestMode){
      P0 = _testModePressureHPaValues[_testModePressureIdx];  //  get first pressure from test values (already in hPa)
      T0 = 10.0;                                            // fixed 10 degrees celcius test value
      _testModePressureIdx++;
    } else {
      P0 = bmp280.readPressure()/100;  // divide by 100 to converto to hPa
      T0 = bmp280.readTemperature();

     _loop = 2;  // global var _loop passes to 2, and never changes again, as we don't change it anymore
    return;
    }
  }

  if (isGpsInit) {  // check GPS info only after GPS was initialized
    // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        double lat = 0.0, lng = 0.0;
        isGpsOk = GpsInfo(&lat, &lng);
        if(isGpsOk){  // change last known position
          latLastValid = lat;  
          lngLastValid = lng;
        }
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS detected: check wiring?"));
      // while(true);
    }
  }

  if( useGpsHw ) {
    ; // already did read above
  } else {
    isGpsOk = true;  // useful for testing
    if(isGpsOk) {
      latLastValid = 37.123456;  // somewhere in Azores
      lngLastValid = -25.675594; // somewhere in Azores
    }
  }

  unsigned long actualTime = millis(); // Returns the number of milliseconds passed since the Arduino board began running the current program. This number will overflow (go back to zero), after approximately 50 days.

  //read sensor values and store in variables
  float T;
  float P;

  if(_inTestMode) {
    P = _testModePressureHPaValues[_testModePressureIdx];  //  get next pressure from test values (already in hPa)
    T = 10.0;                                            // fixed 10 degrees celcius test value

    if(_testModePressureIdx < TEST_MODE_TOT_VALUES -1) {  // if did not reach last valid index (TEST_MODE_TOT_VALUES -1), increase for next cycle

      // time to increase idx to get next pressure from testing values when in test mode? for example each 500 ms
      if (actualTime - _testModeIdxPreviousTime >= TEST_MODE_INTERVAL_MS_READ_PRESSURE ) {
        // save current time for next iteration
        _testModeIdxPreviousTime = actualTime;
        _testModePressureIdx++;
      }
    }
  } else {
    P = bmp280.readPressure() / 100;  // store pressure not in pascal, but in hecto pascal (a common unit used in meteorology for measuring atmospheric pressure)
    T = bmp280.readTemperature();
  }
   
  // calculate altitude in meters
  float H = ((T0 + 273.15) / z) * (1 - (pow(P / P0, (z * R) / g)));

  
  if(previousAltitude == UNINITIALIZED_VELOCITY){  // not yet initilized? If so initialize with current Altitude
    previousAltitude = H;
  }  // as vertical velocity depends on two different Altitudes (per time elapsed), make sure both Altitudes are valid

  // time to calculate velocity? Altitude is based on pressure, and sensor is not too acurate. Velocity is based on Altitudes. Calculate at each INTERVAL_MS_CALC_VELOCITY milliseconds 
  if (actualTime - velocityCalcpreviousTime >= INTERVAL_MS_CALC_VELOCITY) {
    // save current time for next iteration
    velocityCalcpreviousTime = actualTime;
    // Calculate velocity using altitude change over time
    velocity = (H - previousAltitude) / ( (float) INTERVAL_MS_CALC_VELOCITY / (float) 1000.0); // Divide by 1000 to convert milliseconds to seconds
  
    previousAltitude = H;
  }


  switch(velocityStage){
    case Starting:
      Serial.println(F("Initialize GPS"));
      apc220.print(F("Initialize GPS"));  // send to apc220 antenna module
      ss.begin(GPSBaud);
      isGpsInit = true;
      ShowStageChanging(actualTime, velocityStage, InFlight, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
      velocityStage = InFlight;
      break;
    case InFlight:
      if(velocity < DESCENT_RELEVANT_VELOCITY) {  // it is descending, change stage
        ShowStageChanging(actualTime, velocityStage, RelevantDescent, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = RelevantDescent;
        inDescentRelevantVelocityPreviousTime = actualTime;  // save it for next stage code
      }
      break;
    case RelevantDescent:
      if(velocity < DESCENT_RELEVANT_VELOCITY) { // continues descending
        if(actualTime - inDescentRelevantVelocityPreviousTime > intervalMsInRelevantVelocity) {  // let's assure it is descending for at least a given time (few secs?) before changing to next stage
          ShowStageChanging(actualTime, velocityStage, TestingAltitudeAndOpenDoor, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
          velocityStage = TestingAltitudeAndOpenDoor; 
        } else {  // do nothing, continue "waiting" for time in the relevant velocity
          ;
        }
      } else {  // oops, it seems not enough time in relevant descent velocity. Still going up or somewhat horizontal? => move to inflight stage
        ShowStageChanging(actualTime, velocityStage, InFlight, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = InFlight;
      }
      break;
    case TestingAltitudeAndOpenDoor:
      if(H < ALTITUDE_M_TO_OPEN_DOOR){
        // CODE TO OPEN THE DOOR HERE
        servo.attach(ServoPin);  // attaching the servo object created to pin 
        delay(100);
        servo.write(90);  // servo position to open the door
        delay(300);
        servo.detach();

        ShowStageChanging(actualTime, velocityStage, AlreadyOpenedDoor, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
        velocityStage = AlreadyOpenedDoor;
      }
      break;
    case AlreadyOpenedDoor:
      Serial.println(F("Door is already opened. Initialize GPS"));
      apc220.print(F("Door is already opened. Initialize GPS"));  // send to apc220 antenna module
      ss.begin(GPSBaud);
      ShowStageChanging(actualTime, velocityStage, SendDataWithGPSInfo, velocity, DESCENT_RELEVANT_VELOCITY, inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity);
      velocityStage = SendDataWithGPSInfo;
      break;
    case SendDataWithGPSInfo:
      break; // do nothing here, data is sent below
  }

  // send data to antenna from time to time (eg: each 1000 ms)
  char sT_Lat[12]; // temperature and latitude buffer
  char sP_Lng[12]; // pressue and longitude buffer
  char sH[12];

  char sBufferSprintf[61]; // BE CAREFUL, SET BUFFER BIGT ENOUGH FOR DATA IN SPRINTF

  // time to send data?
  if (actualTime - sendDatapreviousTime >= INTERVAL_MS_SEND_DATA) {

    sendDatapreviousTime = actualTime;

    // T, P, H STUFF
    // https://www.programmingelectronics.com/dtostrf/
    dtostrf(T, -8, 2, sT_Lat);   // min width = 8 left aligned, 2 decimals
    dtostrf(P, -8, 2, sP_Lng);
    dtostrf(H, -8, 2, sH);

    sprintf(sBufferSprintf, "t:%lu ; T:%s ; P:%s ; H:%s", millis(), sT_Lat, sP_Lng, sH); // 2 + 10 + 5 + 11 + 5 + 11 + 5 + 11 + 1 = 61  // note: 2 ^ 32 -1 = 4294967295 = 10 digits

    Serial.print(sBufferSprintf);
    apc220.print(sBufferSprintf);  // send to apc220 antenna module

    // GPS has been initialized? Send extra GPS info
    if (isGpsInit) {
      // Send extra GPS info
      dtostrf(latLastValid, 11, 6, sT_Lat);   // min width = 11 left aligned, 6 decimals
      dtostrf(lngLastValid, 11, 6, sP_Lng);

      sprintf(sBufferSprintf, " ; Lt:%s ; Lg:%s", sT_Lat, sP_Lng); // buffer at least 6 + 11 + 6 + 11 + 1 = 35; BE CAREFUL, SET BUFFER BIGT ENOUGH FOR DATA
 
      Serial.print(sBufferSprintf);
      apc220.print(sBufferSprintf);
    }
    Serial.println();
    apc220.println();  // send to apc220 antenna module
  }  // endif send data to antenna from time to time (eg: each 1000 ms)

}

//------------------------------------------------------
// ShowStageChanging( ):
// Every time the program changes stage, it will log the stage it was in,
// to which stage it is changing to, and some extra info that helps the
// team understand WHY it changed to a given stage
//------------------------------------------------------
void ShowStageChanging(unsigned long actualTime, VelocityStageEnum currentVelocityStage, VelocityStageEnum newVelocityStage, 
                       float velocity, float descentRelevantVelocity, 
                       unsigned long inDescentRelevantVelocityPreviousTime, unsigned long intervalMsInRelevantVelocity){
  char sBufferSprintf[250]; // 216 minimum? check below
  char sVelocity[11];
  char sDescentRelevantVelocity[11];

    dtostrf(velocity, -8, 2, sVelocity);   // min width = 8 left aligned, 2 decimals
    dtostrf(descentRelevantVelocity, -8, 2, sDescentRelevantVelocity);

    sprintf(sBufferSprintf, "CHANGING STAGE! t:%lu ; CurrStage:%d ; NewStage:%d ;  Vel:%s ; relVel:%s ; InDescRelVelPrevTime:%lu ; msInRelVel:%lu", 
            millis(), currentVelocityStage, newVelocityStage, 
            sVelocity, sDescentRelevantVelocity,
            inDescentRelevantVelocityPreviousTime, intervalMsInRelevantVelocity); 
            // 18("CHANGING STAGE! t:") + 10(millis) + 13(" ; CurrStage:") + 4(curr stage) + 12 (" ; NewStage:") + 4(new stage) 
            // + 8(" ;  Vel:") + 11 + 10 (" ; relVel:") + 11
            // + 24 (" ; InDescRelVelPrevTime:") + 10 + 14 (" ; msInRelVel:") + 10 + 1  //  // note: 2 ^ 32 -1 = 4294967295 = 10 digits

            // 18 + 10 + 13 + 4 + 12  + 4 + 8 + 11 + 10 + 11 + 24 + 10 + 14 + 10 + 1 = 160  

    Serial.println(sBufferSprintf);
    apc220.println(sBufferSprintf);  // send to apc220 antenna module
}


//------------------------------------------------------
// GpsInfo(double *lat, double *lng):
// Sets the latitude and longitude that is read by the GPS module
// Returns true if values were read and set; false otherwise
//------------------------------------------------------
bool GpsInfo(double *lat, double *lng) {

  bool locationIsValid = false;

  locationIsValid=gps.location.isValid();
	if (locationIsValid) {
    *lat=gps.location.lat();
    *lng=gps.location.lng();
	} else {
    ;
	}


/*
	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid()) {
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	} else {
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid()) {
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	} else {
		Serial.print(F("INVALID"));
	}

	Serial.println();
*/
  return locationIsValid;
}
