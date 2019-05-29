/*
    Robot Power MultiMoto v1.0 demo
    This software is released into the Public Domain
    
    This demo shows the setup of the L9958 chips and runs the 4 motor outputs
    independently.  Also Timer0 and Timer1 are set to higher frequency than standard.
    This may affect the millisecond timer function.
    
    v1.0 - initial release

*/

// include the SPI library:
#include <SPI.h>

// L9958 slave select pins for SPI
#define SS_M4 14
#define SS_M3 13
#define SS_M2 12
#define SS_M1 11

// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7

// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
#define PWM_M4 6     // Timer0

// L9958 Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwm1, pwm2, pwm3, pwm4;
int dir1, dir2, dir3, dir4;


void setup() {

  Serial.begin(9600);
  
  unsigned int configWord;
  
    // put your setup code here, to run once:
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, HIGH);  // HIGH = not selected
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, HIGH);
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, HIGH);
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, HIGH);
  
  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);
  
  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0
  
  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT);  digitalWrite(ENABLE_MOTORS, HIGH);   // HIGH = disabled
  
  /******* Set up L9958 chips *********
  ' L9958 Config Register
  ' Bit
  '0 - RES
  '1 - DR - reset
  '2 - CL_1 - curr limit
  '3 - CL_2 - curr_limit
  '4 - RES
  '5 - RES
  '6 - RES
  '7 - RES
  '8 - VSR - voltage slew rate
  '9 - ISR - current slew rate
  '10 - ISR_DIS - current slew disable
  '11 - OL_ON - open load enable
  '12 - RES
  '13 - RES
  '14 - 0 - always zero
  '15 - 0 - always zero
  */
  
  // set to max current limit and disable ISR slew limiting
  configWord = 0b0000010000001100;
  
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high
  
  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH); 
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);
  
  // Reduce the PWM frequency to about 8kHz
  // Note, this will screw up the timer functions that use Timer0 such as millis()
  setPwmFrequency(PWM_M1, 8);
  setPwmFrequency(PWM_M3, 8);
}


// *******************************************
// ************** Main Loop ******************
// *******************************************
void loop() {

  while (Serial.available() > 0) {
    int LED_1 = Serial.parseInt();
    int LED_2 = Serial.parseInt();

//    LED_1 and LED_2 should be from 0 - 255
    analogWrite(PWM_M1, LED_1);  digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M2, LED_2);  digitalWrite(DIR_M2, dir1);

//    Serial.print(LED_1);
//    Serial.print(LED_2);

    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
    delay(100);
  }
//  // put your main code here, to run repeatedly:
//
//  //dir1 = 1; dir2 = 0; dir3 = 0; dir4 = 0;
//  //pwm1 = 47; pwm2 = 154; pwm3 = 225; pwm4 = 3;
//  /*
//  pwm1 = random(256); pwm2 = random(256); pwm3 = random(256); pwm4 = random(256);
//  dir1 = random(2); dir2 = random(2); dir3 = random(2); dir4 = random(2);
//  */
// 
//  // toggle direction 
//  dir1 = !dir1;
//  
//  // ramp speed up for all motors
//  for (pwm1 = 0; pwm1 < 256; pwm1 +=5) {
//    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
//    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
//    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
//    analogWrite(PWM_M4, pwm1);  digitalWrite(DIR_M4, dir1);
//    
//    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
//    delay(500);
//  }
//  delay(8000); // delay 2 sec
//  
//  // ramp speed down for all motors
//  for (pwm1 = 255; pwm1 >= 0; pwm1 -=5) {
//    analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
//    analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
//    analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
//    analogWrite(PWM_M4, pwm1);  digitalWrite(DIR_M4, dir1);
//    
//    digitalWrite(ENABLE_MOTORS, LOW);  // enable = LOW
//    delay(500);
//  } 
//  delay(8000);  // this is actually about 2 sec because we've changed Timer0's rate for the PWM

}
// ***********************************************
// ************** End Main Loop ******************
// ***********************************************


/*
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired (Timer0)
 *   - Pins 9 and 10 are paired (Timer1)
 *   - Pins 3 and 11 are paired (Timer2)
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 5, 6 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
 
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) { // Timer0 or Timer1
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) { 
      TCCR0B = TCCR0B & 0b11111000 | mode; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode; // Timer1
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode; // Timer2
  }
}
