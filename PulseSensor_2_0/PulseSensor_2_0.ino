:/*
 * Pulse detecting program based on the pulse sensor (https://pulsesensor.com/) example:
 * https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino
 * 
 * This program collect samples from each peak-to-peak data to make a dataset which is
 * later used to calculate the mean and standard deviations. The mean and stardard deviations
 * are used to determine how a normal heartbeat signal should be for a user.
 * 
 * This program also uses the EMA (Exponential Moving Average) filter to smooth out the signal.
 * The library can be found here: https://github.com/sofian/MovingAverage
 */

#include <MovingAverage.h>
#include <math.h>

#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER  2

#define Samples 19                // The number of sample to collect within peak-to-peak data
#define dataPoints 20             // The number of data points to collect to complete a dataset

// The thresholds to decide whether the heartrate is normal enough to record
#define maxHeartrate 120
#define minHeartrate 40

// A constant that a counter use to ignore the first few messy heartrate
#define HEARTBEAT_COUNT 10

#define PWM_base 255/2            // 50% duty cycle
#define PWM_low 255/5             // 20% duty cycle
#define PWM_high 255*4/5          // 80% duty cycle

//  Variables
int pulsePin = A0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin
short maxNum = 0;
short points_difference = 0;      // The distance between 2 sample points
short datasetIndex = 0;           // The index of to keep track of the collected data points
short safeUpperHeartrate = 0;     // The upper bound of the heartrate range of the user
short safeLowerHeartrate = 0;     // The lower bound of the heartrate range of the user

const int maxPeakIndex = 60000/(2*minHeartrate);      // The maximum index of the peak_to_peak array based on the minimum normal heartrate
const int minPeakIndex = 60000/(2*maxHeartrate);      // The minimum index of the peak_to_peak array based on the maximum normal heartrate

boolean collecting = true;                            // The flag to make sure the program is only collecting data
boolean calibrating = false;                          // This flag is true when the collecting phase is done and it's ok to do calculations

volatile short *peak_to_peak;                     // The array to collect all nanlog data between 2 peaks
                                                  // peak_to_peak[0] holds the size of the collected array
short dataset [dataPoints][Samples + 1];          // The array holds the data set to calculate mean and standard deviation
                                                  // dataset[][last_index] holds the differences between 2 data points
float usage [Samples + 1][2];                     // The array saves the calculate mean and standard deviations for usage
                                                  // usage[last_index][] holds mean and standard deviation of the differences

// Volatile Variables, used in the interrupt service routine!
volatile int heartbeatCounter = 0;      // Count the heartbeats. It is used for the program to ignore the first few heartbeats
volatile int BPM;                       // int that holds raw Analog in 0. updated every 2mS
volatile short Signal;                  // holds the incoming raw data
volatile int IBI = 600;                 // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;         // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;            // becomes true when Arduoino finds a beat.
volatile boolean startSaving = false;   // The flag is true when the peak-to-peak data is finished collecting

static int outputType = SERIAL_PLOTTER;

// Exponential moving average filter
// We can control the smoothness of the waveform by changing the α value in the parentheses
// α = 2 / (N+1)
// N samples
MovingAverage average(0.1f);

//FilterOnePole filterOneLowpass( LOWPASS, 7 );

void setPwmFrequency(int pin, int prescaler);

void setup() {
  pinMode(blinkPin, OUTPUT);        // pin that will blink to your heartbeat!
  pinMode(fadePin, OUTPUT);         // pin that will fade to your heartbeat!
  Serial.begin(9600);               // we agree to talk fast!
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS

  peak_to_peak = malloc(sizeof(short) * maxPeakIndex);

  // Initialize arrays to 0
  int i, j;
  for (i = 0; i < maxPeakIndex; i++) {
    peak_to_peak[i] = 0;
  }
  for (i = 0; i < dataPoints; i++) {
    for (j = 0; j < Samples + 1; j++) {
      dataset[i][j] = 0;
    }
  }
  for (i = 0; i < Samples + 1; i++) {
    for (j = 0; j < 2; j++) {
      usage[i][j] = 0;
    }
  }

  average.reset( analogRead(pulsePin) );

  //timeStamp = millis();

  // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
  // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //   analogReference(EXTERNAL);
}

void loop() {

  serialOutput() ;
  if (QS == true) {    // A Heartbeat Was Found
    // BPM and IBI have been Determined
    // Quantified Self "QS" true when arduino finds a heartbeat
    fadeRate = 255;         // Makes the LED Fade Effect Happen

    // Keep incrementing the counter and let it stops at 10
    heartbeatCounter = heartbeatCounter + 1;
    if (heartbeatCounter > 10) {
      heartbeatCounter = 10;
    }
    Serial.println("Heartbeat detected");

    // After calulations are completed, we can use the calculated heartbeat range to dedect whether the user is in danger
    if (safeUpperHeartrate != 0 && safeLowerHeartrate != 0) {
      if (BPM < safeLowerHeartrate || BPM > safeUpperHeartrate) {
        // Send braking signal to the wheelchair
        Serial.println("STOP");
        //analogWrite(7, PWM_base);
        //analogWrite(6, PWM_base);
      }
    }
    
    // Set 'fadeRate' Variable to 255 to fade LED with pulse
    serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
    QS = false;                      // reset the Quantified Self flag for next time
  }

  // The collecting phase
  if (collecting == true && calibrating == false && peak_to_peak[0] != 0) {
    if (startSaving == true) {
      // Calculate the distance between 2 sample points
      points_difference = peak_to_peak[0] / Samples;    
                  
      // Put the sample points and calculated data into the dataset array
      for (int i = 0; i < Samples; i++) {
        dataset[datasetIndex][i] = peak_to_peak[i * Samples];
      }
      dataset[datasetIndex][Samples] = points_difference;

      // Move to next data point and reset the saving flag
      datasetIndex = datasetIndex + 1;
      startSaving = false;
    }
  }  else if (calibrating == true && collecting == false) {    // The calculations for the mean and standard deviations of the dataset
    free(peak_to_peak);
    long int sum = 0;
    float average = 0;
    float sqDevSum = 0.0;
    float stDev = 0.0;

    for (int i = 0; i < Samples; i++) {
      sum = 0;
      sqDevSum = 0.0;
      for (int j = 0; j < dataPoints; j++) {
        sum = sum + dataset[j][i];
      }
      average = float(sum) / float(dataPoints);

      for (int j = 0; j < dataPoints; j++) {
        // pow(x, 2) is x squared.
        sqDevSum += pow((float(dataset[j][i]) - average), 2);
      }
      stDev = sqrt(sqDevSum / float(dataPoints));

      // Save the mean and standard deviation
      usage[i][0] = average;
      usage[i][1] = stDev;
    }
    // Calculate mean and standard deviation of difference(s)
    sum = 0;
    sqDevSum = 0.0;
    for (int i = 0; i < dataPoints; i++) {
      sum = sum + dataset[i][Samples];
    }
    average = float(sum) / float(dataPoints);

    for (int i = 0; i < dataPoints; i++) {
      // pow(x, 2) is x squared.
      sqDevSum += pow((float(dataset[i][Samples]) - average), 2);
    }
    stDev = sqrt(sqDevSum / float(dataPoints));
    usage[Samples][0] = average;
    usage[Samples][1] = stDev;
    
    Serial.println("Calculations finished");
    safeUpperHeartrate = 60000/((2*(average - stDev)) * Samples);
    safeLowerHeartrate = 60000/((2*(average + stDev)) * Samples);
    calibrating = false;
    datasetIndex = dataPoints + 10;
  }

  // When the program collected enough data, it switches to calculation phase
  if (datasetIndex == dataPoints) {
    collecting = false;
    calibrating = true;
  } else if (datasetIndex > dataPoints) {
    datasetIndex = dataPoints + 10;
  }
  
  ledFadeToBeat();                      // Makes the LED Fade Effect Happen
  delay(20);                             //  take a break
}

void ledFadeToBeat() {
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate, 0, 255); //  keep LED fade value from going into negative numbers!
  analogWrite(fadePin, fadeRate);         //  fade LED
}

void setPwmFrequency(int pin, int prescaler) {
  byte mode;
  if(pin >= 2 && pin <= 13) {
    switch(prescaler) {
      case 1: mode = 0x01; break;
      case 2: mode = 0x02; break;
      case 3: mode = 0x03; break;
      case 4: mode = 0x04; break;
      case 5: mode = 0x05; break;
      case 6: if (pin == 9 || pin == 10) {mode = 0x06;} break;
      case 7: if (pin == 9 || pin == 10) {mode = 0x07;} break;
      default: return;
    }
    if(pin == 13 || pin == 4) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else if(pin == 12 || pin == 11) {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    } else if(pin == 10 || pin == 9) {
      TCCR2B = TCCR2B & 0b11111000 | mode;
    } else if(pin == 5 || pin == 3 || pin == 2) {
      TCCR3B = TCCR3B & 0b11111000 | mode;
    } else if(pin == 8 || pin == 7 || pin == 6) {
      TCCR4B = TCCR4B & 0b11111000 | mode;
    }
  }
}
