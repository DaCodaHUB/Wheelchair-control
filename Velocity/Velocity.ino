/*
 * Filters: Smoothing (10 points), (Low pass EMA)
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <math.h>
//#include <Filters.h>

#define ELEMENTS 3

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

// Delay
int deltat = 0;
int start = 0;

// Velocity
float curSpeed = 0;
float lastSpeed = 0;

// Accelerations
sensors_event_t event;
float accelX = 0;
float accelY = 0;
float accelZ = 0;
float accelTotal = 0;

// Smoothing
const int numReadings = 10;
int readings[numReadings * ELEMENTS];      // the readings from the analog input
int readIndex[ELEMENTS];              // the index of the current reading
int total[ELEMENTS];                  // the running total
int average[ELEMENTS];                // the average

int Smoothing (int eleNum, int data);
void displaySensorDetails(void);

void setup(void)
{
  Serial.begin(9600);
  for (int i = 0; i < (numReadings * ELEMENTS); i++) {
    readings[i] = 0;
  }
  for (int i = 0; i < ELEMENTS; i++) {
    readIndex[i] = 0;
    total[i] = 0;
    average[i] = 0;
  }
  
  Serial.println(F("Adafruit 9DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void)
{
/*
  // Display the results (acceleration is measured in m/s^2)
  accel.getEvent(&event);
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(Smoothing(0, event.acceleration.x)); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Smoothing(1, event.acceleration.y)); Serial.print("  ");
  Serial.print("Z: "); Serial.print(Smoothing(2, event.acceleration.z)); Serial.print("  ");Serial.println("m/s^2 ");
  Serial.println(F(""));
  delay(50);
*/

  deltat = millis() - start;
  if (deltat > 10) {
    accel.getEvent(&event);
    
    //accelX = event.acceleration.x;
    //accelY = event.acceleration.y;
    //accelZ = event.acceleration.z;
    //start = millis();
    
    accelX = Smoothing(0, event.acceleration.x);
    accelY = Smoothing(1, event.acceleration.y);
    accelZ = Smoothing(2, event.acceleration.z);
    start = millis();
    accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ) - 10.70;
    if (accelTotal > 0) {
      curSpeed = accelTotal * 0.01 + lastSpeed;
      lastSpeed = curSpeed;
    } else if ((accelX < 0 && accelY < 0 && accelZ >= 9.8 && accelTotal > 0) ||(accelX < 0 && accelY < 0 && accelZ < 9.8 && accelTotal < -0.5)) {
      curSpeed = lastSpeed - accelTotal * 0.01;
      lastSpeed = curSpeed;
    } else {
      lastSpeed = 0;
    }
  }
  //Serial.print("Acceleration ");
  //Serial.print(accelTotal);
  //Serial.print(" Speed ");
  //Serial.println(curSpeed);

}

int Smoothing (int eleNum, int data) {
  int readingIndex = (eleNum * numReadings) + readIndex[eleNum];
  // subtract the last reading:
  total[eleNum] = total[eleNum] - readings[readingIndex];
  // read from the sensor:
  readings[readingIndex] = data;
  // add the reading to the total:
  total[eleNum] = total[eleNum] + readings[readingIndex];
  // advance to the next position in the array:
  readIndex[eleNum] = readIndex[eleNum] + 1;
  //Serial.println(readIndex[eleNum]);
  //Serial.println(readingIndex);
  //Serial.println(total[eleNum]);
  
  // if we're at the end of the array...
  if (readIndex[eleNum] >= numReadings) {
    // ...wrap around to the beginning:
    readIndex[eleNum] = 0;
  }

  // calculate the average:
  average[eleNum] = total[eleNum] / numReadings;
  return average[eleNum];
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);

  start = millis();
}

