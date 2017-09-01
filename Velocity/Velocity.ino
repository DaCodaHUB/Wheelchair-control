/*
 * Author: Dang Le
 * Using IMU BNO055
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
//#include <Filters.h>

#define ELEMENTS 3

/* Assign a unique ID to the sensors */
Adafruit_BNO055 bno = Adafruit_BNO055();

// Delay
int deltat = 0;
int start = 0;

// Velocity
float curSpeed = 0;
float lastSpeed = 0;

// Accelerations
sensors_event_t event;
float accel[3];
//float lastAccel[3];
float accelTotal = 0;
float lastAccelTotal = 0;

// Smoothing
const int numReadings = 10;
int readings[numReadings * ELEMENTS];      // the readings from the analog input
int readIndex[ELEMENTS];              // the index of the current reading
int total[ELEMENTS];                  // the running total
int average[ELEMENTS];                // the average

boolean slowDown = false;

int Smoothing (int eleNum, int data);

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
    accel[i] = 0;
    //lastAccel[i] = 0;
  }

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  
  bno.setExtCrystalUse(true);
}

void loop(void)
{
  deltat = millis() - start;
  if (deltat > 10) {
    imu::Vector<3> lineaeAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accel[0] = Smoothing(0, lineaeAccel.x());
    accel[1] = Smoothing(1, lineaeAccel.y());
    accel[2] = Smoothing(2, lineaeAccel.z());
    start = millis();
    /*
    Serial.print("X: ");
    Serial.print(accel[0]);
    Serial.print(" Y: ");
    Serial.print(accel[1]);
    Serial.print(" Z: ");
    Serial.print(accel[2]);
    Serial.print("\t\t");
    */
    accelTotal = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if (accelTotal < lastAccelTotal) {
      slowDown = true;
    } else if (accelTotal > lastAccelTotal) {
      slowDown = false;
    }

    if (slowDown == false) {
      curSpeed = lastSpeed + accelTotal * 0.01;
    } else {
      if (accelTotal > 0) {
        curSpeed = lastSpeed - accelTotal * 0.01;
        if (curSpeed < 0) {
          curSpeed = 0;
        }
      } else {
        curSpeed = 0;
        slowDown = false;
      }
    }
    lastSpeed = curSpeed;
    lastAccelTotal = accelTotal;
  }
  Serial.print("Acceleration ");
  Serial.print(accelTotal);
  Serial.print(" m/s^2 ");
  Serial.print("Speed ");
  Serial.print(curSpeed);
  Serial.println(" m/s");
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
  
  // if we're at the end of the array...
  if (readIndex[eleNum] >= numReadings) {
    // ...wrap around to the beginning:
    readIndex[eleNum] = 0;
  }

  // calculate the average:
  average[eleNum] = total[eleNum] / numReadings;
  return average[eleNum];
}

