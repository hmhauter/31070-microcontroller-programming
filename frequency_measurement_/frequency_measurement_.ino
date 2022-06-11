#include "Timer5.h"
#include <LiquidCrystal.h>
#include <math.h>
#define PI 3.1415926535897932384626433832795
LiquidCrystal lcd(12, 4, 5, 11, 3, 2);

// Global Variables

// Debugging 
const int ledPin = 1;

// Frequency Calculation
volatile long int zeroCrossing = 0;

float sample = 0.0;
volatile float frequency = 0.0;
volatile float debug =0.0;
const float samplingRate = 10000.0;
const float offsetValueZeroCrossing = 1100; // 257.12;

const int resolution = 4095; 

// Low Pass Filter
const float crossOverFreq = 50.0;


// float alpha;
// float alpha = 0.03045902795;      // from own calculation 
float alpha = 0.027947230169320696;  //50 Hz - 10 ksps in interrupt (from lecture)
// float alpha_new = 0.006243953391; // 10Hz
// float alpha_new = 0.05911739744;  // 100 Hz
// float alpha_new = 0.015465039;    // sampling rate 20000 Hz
// float oneMinusAlpha_new = 1.0-alpha_new; 
float oneMinusAlpha = 1.0-alpha; //Calculation to speed up filter calculation in ISR

volatile float oldY = 0.0;
volatile float newY = 0.0;

volatile float newT = 0.0;
volatile float oldT = 0.0;

volatile float voltage = 0.0;
volatile float ampere = 0.0;
volatile float RMS_voltage = 0.0;
volatile float RMS_ampere = 0.0;
volatile float sampleCounter = 0.0;
volatile float power = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // alpha = calculateAlpha(crossOverFreq, (float)(1.0/samplingRate));
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  analogWrite(7, 1);

  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(calculateFrequency);
  AdcBooster();
  // DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
}

void loop() {
  Serial.println("Frequency: ");
  Serial.println(frequency, 6);
  Serial.println("RMS Ampere: ");
  Serial.println(RMS_ampere, 6);
  Serial.println("RMS Voltage: ");
  Serial.println(RMS_voltage, 6);
    Serial.println("Power: ");
  Serial.println(power, 6);
    digitalWrite(ledPin, 1);
    if (zeroCrossing % 20 == 0) {
      lcd.clear();
      lcd.print((frequency), 6);
      }
    if (frequency <= 49.9 || frequency >= 50.2 ) {
        digitalWrite(6, 1);
        }
        else {
          digitalWrite(6,0);
          }

  // put your main code here, to run repeatedly:
  // int sample = analogRead(A1);
  // float volt = (3.3/1023) * sample;
  // newY = lowPassFilter(sample, oldY);
  // oldY = newY;
  // digitalWrite(A0, newY);
  // Serial.println(volt);
  //delay(1);

}

void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16.
  ADC_CTRLB_RESSEL_12BIT; // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
  ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00; // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
} 

float lowPassFilter(float Input, float yOld){
  //Low-pass filter function
  return (float) alpha*Input+oneMinusAlpha*yOld;  
} 

void calculateFrequency(void) {
  // Interrupt start
  // Calculate Frequency
  analogReadResolution(12);
  analogWriteResolution(12);
  sample = (float)analogRead(A1);
  newY = lowPassFilter(sample, oldY);
  
  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing and oldY < offsetValueZeroCrossing) {
    // detectZeroCrossing();
    newT = millis()/1000.0;
    debug = ((newY - offsetValueZeroCrossing) / (newY - oldY)) * (1.0/samplingRate);
    float period = newT - oldT - debug;
    frequency = (1.0/period);
    oldT = newT;
    zeroCrossing++;
    RMS_voltage = sqrt((1/sampleCounter) * voltage);
    RMS_ampere = sqrt((1/sampleCounter) * ampere);
    power = RMS_voltage * RMS_ampere;
    voltage = 0.0;
    ampere = 0.0;
    sampleCounter = 0.0;
    }
    else {
//      Serial.println("Sample: ");
//      Serial.println(sample);
//      Serial.println("Mapping Volt: ");
//      Serial.println(map(sample, 0, resolution, -240*sqrt(2), 240*sqrt(2)));
      voltage = voltage + powf(map(sample, 0, resolution, -240*sqrt(2), 240*sqrt(2)), 2);
      ampere = ampere + powf(map(sample, 0, resolution, -42*sqrt(2), 42*sqrt(2)), 2);
      sampleCounter++;
      
      }
  oldY = newY;
  analogWrite(A0, newY);
    
}

void detectZeroCrossing() {
  // debug = ((newY*(1.0/samplingRate))/(newY-oldY));
  // debug = (offsetValueZeroCrossing - oldY) * ((1.0/samplingRate)/(newY-oldY));
  debug = ((newY - offsetValueZeroCrossing) / (newY - oldY)) * (1.0/samplingRate);
  
  // debug = ((newY - offsetValueZeroCrossing) *((1.0/samplingRate)/(newY-oldY)));
   // float period = (sampleCount/samplingRate)-((newY*(1.0/samplingRate))/(newY-oldY));
   newT = millis()/1000.0;
   
   float period = newT - oldT; //- debug;
//Serial.println("New T : ");
//Serial.println(newT);
//Serial.println("Old T : ");
//Serial.println(oldT);
// Serial.println("Debug  : ");
// Serial.println(debug, 6);
//  Serial.println("Interpol: ");
//  Serial.println(((newY*(1/samplingRate))/(newY-oldY)), 5);
// Serial.println("Period: ");
// Serial.println(period, 6);
//    Serial.println("Y new: ");
//  Serial.println(newY);
//    Serial.println("Y old: ");
//  Serial.println(oldY);
  frequency = ((float) 1.0/period);
  oldT = newT;
//  Serial.println(zeroCrossing);
//  Serial.println(samplingRate);
//  Serial.println(frequency);
  }

float calculateAlpha(float crossOverFreq, float deltaT) {
  float RC = 1 / (2 * PI * crossOverFreq);
  return deltaT / (RC + deltaT);
  }

  
