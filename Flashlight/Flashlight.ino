// Flashlight implements logic for an accelerometer controlled dual color flashlight.
//  Copyright (C) 2014 Nicola Cimmino
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include <EEPROM.h>

// X-Axis is vertical when holding the device sraight.
#define ACC_X_PIN A0

// We use an IIR (Infinite Impulse Response) filter to filter the accelleration.
// The filter taps have been calculated in Octave.
// The Low Pass filter is set for:
// Sampling frequency: 100Hz
// Cut-off frequency: 3Hz (3 dB)
// Order: 3rd
// 
// Below we define this filter forward and feedback taps and the circular buffer used to hold the samples.
#define accellFilterTapsCount 4
float accellFilterForward_Taps[accellFilterTapsCount] = { 6.9935e-04, 2.0980e-03, 2.0980e-03, 6.9935e-04 };
float accellFilterFeedback_Taps[accellFilterTapsCount] = {1.00000,  -2.62355,   2.31468,  -0.68554 };
float accellInputSamples[accellFilterTapsCount];
float accellOutputSamples[accellFilterTapsCount];
int accellSamplesBufferIndex = 0;

// Interval between accelerometer samples in mS. This is a sampling frequency of 100Hz
#define accellSamplingInterval 10

// Stores the timestamp of the last sample so that the next can be taken at the exact needed moment.
long lastSampleTime = 0;

// Amount of samples averaged during calibration
#define calibrationSamples 20

// Pins assignement.
#define ACCELEROMETER_PWR 13
#define ACCELEROMETER_GND A3
#define BUTTON 3
#define LED_WHITE_A 10
#define LED_WHITE_K 11
#define LED_RED_A 9
#define LED_RED_K 8

// EEPROM map.
#define EEPROM_CALIBRAION_BASE 0

// Possible modes.
// MODES_COUNT must always left last.
enum modes { STEADY, BEACON, STROBE, MODES_COUNT } ;

// The current light mode.
modes currentMode = STEADY;

// Currently operative LED.
int currentLED = LED_RED_A;

// Current LED intensity (affects all modes).
byte currentIntensity = 50;

// Interval between strobe pulses, in mS.
long strobe_interval = 1000;

// Duration of the on phase of the beacon.
long beacon_ontime = 500;

void setup(){

  // Power up the acceletometer
  pinMode(ACCELEROMETER_PWR, OUTPUT);
  digitalWrite(ACCELEROMETER_PWR, HIGH);
  pinMode(ACCELEROMETER_GND, OUTPUT);
  digitalWrite(ACCELEROMETER_GND, LOW);
   
  // Accelerometer outputs
  pinMode(ACC_X_PIN, INPUT);
 
  // Set button pin as inpur and activate internal pull up.
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
    
  pinMode(LED_WHITE_K, OUTPUT);
  digitalWrite(LED_WHITE_K, LOW);
  pinMode(LED_RED_K, OUTPUT);
  digitalWrite(LED_RED_K, LOW);
  pinMode(LED_WHITE_A, OUTPUT);
  analogWrite(LED_WHITE_A, 0);
  pinMode(LED_RED_A, OUTPUT);
  analogWrite(LED_RED_A, 0);
  
  for(int sampleIndex=0; sampleIndex<accellFilterTapsCount; sampleIndex++)
  {
    accellInputSamples[sampleIndex] = 0;
    accellOutputSamples[sampleIndex] = 0;
  }

  Serial.begin(9600);
 
}

void calibrate()
{
  Serial.println("Calibrating....");
  Serial.println("Turn device wih X axis facing down and then slowly turn it with X asis facing up.");
  
  long startTime = millis();  
  
  int maxValue = 0;
  int minValue = 1024;
  
  // Calibrate for 3s
  while(millis() - startTime < 3000)
  {
    long total = 0;
    for(int sample=0; sample<calibrationSamples; sample++) {
      total += analogRead(ACC_X_PIN)/2;
      delay(accellSamplingInterval);
    }
    int average = total/calibrationSamples;      
    if(average > maxValue) {
      maxValue = average;
    }
    
    if(average < minValue) {
      minValue = average;
    }
  }
    
  // Store in EEPROM. We use locations from EEPROM_CALIBRAION_BASE as:
  // ZeroX, KX
  // Where ZeroX is the value corresponding to 0g on the X axis and
  // Kx is the change in value representing 1g or, in other words, the slope.
  EEPROM.write(EEPROM_CALIBRAION_BASE, (minValue+((maxValue-minValue)/2))); 
  EEPROM.write(EEPROM_CALIBRAION_BASE+1, ((maxValue-minValue)/2));

  Serial.println(EEPROM.read(EEPROM_CALIBRAION_BASE));
  Serial.println(EEPROM.read(EEPROM_CALIBRAION_BASE+1));
  
  Serial.println("Calibration done.");
    
}


void loop(){
 
  // Sit here forever waitig user to press the button.
  waitButtonDown(0);
  
  // If the button is released within 500mS change mode
  //  otherwise start the mode adjustement.
  if(waitButtonUp(500))
  {
    changeMode();  
  }
  else
  {
    adjustMode();  
  }   
}

/*
 * When invoked this function will continuosly monitor the acceleometer and
 * apply adjustments to the current mode depending on the tilt. This function
 * will return only when the button is released.
 */
void adjustMode()
{
  while(digitalRead(BUTTON)==LOW)
  {
    // Wait right time for sampling, this is important as the 
    //  filter characteristics depend on the sampling frequency.
    while(millis() - lastSampleTime < accellSamplingInterval)
    {
      delay(1);
    }
    lastSampleTime = millis();
      
    // Get the current reading and convert to g according to the calibration table.
    float currentReading = analogRead(ACC_X_PIN)/2;
    currentReading = (currentReading-(EEPROM.read(EEPROM_CALIBRAION_BASE)))/(float)EEPROM.read(EEPROM_CALIBRAION_BASE+1);
    
    // Store the current reading at the current position of the circular buffer.
    accellInputSamples[accellSamplesBufferIndex] = currentReading; 
     
    // Calculate the IIR filter output.
    float accellOutput = 0;
    
    int sampleIndex = accellSamplesBufferIndex;
    for(int ix=0;ix<accellFilterTapsCount;ix++) {
      accellOutput += accellInputSamples[sampleIndex] * accellFilterForward_Taps[ix];
      if(ix>0) accellOutput -= accellOutputSamples[sampleIndex] * accellFilterFeedback_Taps[ix];
      sampleIndex = (sampleIndex>0)?sampleIndex-1:accellFilterTapsCount-1;
    }
    accellOutputSamples[accellSamplesBufferIndex] = accellOutput;
    
    // Move to the next  position of the circular buffer and wrap around if we are
    // at the end of the array.  
    accellSamplesBufferIndex = (accellSamplesBufferIndex + 1) % accellFilterTapsCount;

    // We ignore the direction of the accelleration vector,
    // we just care to know the tilt (1g=horizontal 0g=vertical).
    currentReading=abs(accellOutput);
    
    if(currentMode == STEADY)
    {
      currentIntensity = (currentReading)*255;
    }
    else if(currentMode == BEACON)
    {
      beacon_ontime = (currentReading)*900;
    }
    else if(currentMode == STROBE)
    {
      strobe_interval = (currentReading)*1000;  
    }
    //analogWrite(currentLED, currentIntensity);
    lightsTick();
  }  
}

/*
 * When invoked will change the mode to the next.
 * All modes are cycled and when the cycle is repeated
 * the current active LED is changed.
 */
void changeMode()
{
    currentMode = (modes)(((int)currentMode + 1) % MODES_COUNT);
    if(currentMode == 0)
    {
      if(currentLED==LED_RED_A)
      {
        currentLED=LED_WHITE_A;
      }
      else
      {
        currentLED=LED_RED_A;
      }
    }
}

/*
 * Waits until the button is released or the specified timeout occours.
 * Returns true if the button was released. 
 */
boolean waitButtonUp(long timeout)
{
  return waitButton(/*waitForHigh=*/true, timeout);
}

/*
 * Waits until the button is pressed or the specified timeout occours.
 * Returns true if the button was pressed. 
 */
boolean waitButtonDown(long timeout)
{
  return waitButton(/*waitForHigh=*/false, timeout);
}

/*
 * Waits until the desired button condition or the specified timeout occours.
 * Returns true if the button was released. 
 */
boolean waitButton(boolean waitForHigh, long timeout)
{
  long startTime = millis();
  while(digitalRead(BUTTON)==(waitForHigh)?LOW:HIGH && (timeout==0 || millis()-startTime<timeout))
  {
    delay(1);
    lightsTick();
  }
  
  return digitalRead(BUTTON)==(waitForHigh)?HIGH:LOW;
}

/*
 * Must be called regularly to allow dynamic light modes to work.
 * Frequency is not critical as PWM is not done here. Modes are timed
 * according to timers and not calls frequency.
 */
void lightsTick()
{
  if(currentMode==STEADY)
  {
    // Steady light at the current intensity.
    analogWrite(currentLED, currentIntensity);   
  }
  else if(currentMode==BEACON)
  {
    // 
    analogWrite(currentLED, (millis()%1000<beacon_ontime)?currentIntensity:0);  
  }
  else if(currentMode==STROBE)
  {
    analogWrite(currentLED, (millis()%strobe_interval<50)?currentIntensity:0);  
  }
}
