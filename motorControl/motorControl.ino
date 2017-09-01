/*
 * PWM frequency for MEGA
 * 
 * timer 0 (controls pin 13, 4);
 * timer 1 (controls pin 12, 11);
 * timer 2 (controls pin 10, 9);
 * timer 3 (controls pin 5, 3, 2);
 * timer 4 (controls pin 8, 7, 6);
 * 
 * timers (TCCR1B, TCCR2B, TCCR3B, TCCR4B):
 * prescaler = 1 ---> PWM frequency is 31000 Hz
 * prescaler = 2 ---> PWM frequency is 4000 Hz
 * prescaler = 3 ---> PWM frequency is 490 Hz (default value)
 * prescaler = 4 ---> PWM frequency is 120 Hz
 * prescaler = 5 ---> PWM frequency is 30 Hz
 * prescaler = 6 ---> PWM frequency is <20 Hz
 * 
 * timer 0 (TCCR0B):
 * prescaler = 1 ---> PWM frequency is 62000 Hz
 * prescaler = 2 ---> PWM frequency is 7800 Hz
 * prescaler = 3 ---> PWM frequency is 980 Hz (default value)
 * prescaler = 4 ---> PWM frequency is 250 Hz
 * prescaler = 5 ---> PWM frequency is 60 Hz
 * prescaler = 6 ---> PWM frequency is <20 Hz
 */

/*
 * PWM:
 * 9: left, right
 * 10: front, back
 * 
 * Buttons:
 * 48: front
 * 47: back
 * 49: right
 * 46: left
 * 
 * Potentiometer: speed - A8
 * Map: val*153/2046
 */

/*
 * Bugs:
 * Switch to timer2 using 9 and 10
 */
#define PWM_base 255/2
#define PWM_low 255/5
#define PWM_high 255*4/5

// Keypress with digitalRead
int emergencyBrake = 0;

// Potentiometer with analogRead
int voltageRate = 0;
const int frontButton = 48;
const int backButton = 47;
const int rightButton = 49;
const int leftButton = 46;

// Movements
boolean goRight = false;
boolean goLeft = false;
boolean goForward = false;
boolean goBackward = false;

void setPwmFrequency(int pin, int prescaler);

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  pinMode(frontButton, INPUT);
  pinMode(backButton, INPUT);
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  setPwmFrequency(9, 3);
  setPwmFrequency(10, 3);
}

// the loop routine runs over and over again forever:
void loop() {
  int val = analogRead(A8);
  int regularizeOutValue = 0;
  voltageRate = map(val, 0, 1023, 0, PWM_base - PWM_low);
  
  if (digitalRead(frontButton) == HIGH) {
    goForward = true;
  } else {
    goForward = false;
  }

  if (digitalRead(backButton) == HIGH) {
    goBackward = true;
  } else {
    goBackward = false;
  }

  if (digitalRead(rightButton) == HIGH) {
    goRight = true;
  } else {
    goRight = false;
  }

  if (digitalRead(leftButton) == HIGH) {
    goLeft = true;
  } else {
    goLeft = false;
  }

  if (goForward == true) {
    regularizeOutValue = PWM_base + voltageRate;
    if(regularizeOutValue <= PWM_high){
      Serial.println ("Go forward");
      analogWrite(10, regularizeOutValue);
      analogWrite(9, PWM_base);
    }else{
      //whoops! Something went wrong and my values didn't fit within thresholds expected.
      //write this out to serialOut and figure out what edge case we didn't handle
    }    
  } else if (goBackward == true) {
    regularizeOutValue = PWM_base - voltageRate;
    if(regularizeOutValue >= PWM_low){
      Serial.println ("Go backward");
      analogWrite(10, regularizeOutValue);
      analogWrite(9, PWM_base);
    }else{
      //whoops! Something went wrong and my values didn't fit within thresholds expected.
      //write this out to serialOut and figure out what edge case we didn't handle
    } 
  } else if (goRight == true) {
    regularizeOutValue = PWM_base + voltageRate;
    if(regularizeOutValue <= PWM_high){
      Serial.println ("Go right");
      analogWrite(9, regularizeOutValue);
      analogWrite(10, PWM_base);
    }else{
      //whoops! Something went wrong and my values didn't fit within thresholds expected.
      //write this out to serialOut and figure out what edge case we didn't handle
    } 
  } else if (goLeft == true) {
    regularizeOutValue = PWM_base - voltageRate;
    if(regularizeOutValue >= PWM_low){
      Serial.println ("Go left");
      analogWrite(9, regularizeOutValue);
      analogWrite(10, PWM_base);
    }else{
      //whoops! Something went wrong and my values didn't fit within thresholds expected.
      //write this out to serialOut and figure out what edge case we didn't handle
    } 
  } else {
    analogWrite(10, PWM_base);
    analogWrite(9, PWM_base);
  }
  delay(30);
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
