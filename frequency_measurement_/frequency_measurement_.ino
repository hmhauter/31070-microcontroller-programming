#include "Timer5.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 4, 5, 11, 3, 2);

// Global Variables

// Debugging 
const int ledPin = 1;

// Low Pass Filter
float alpha = 0.03045902795;      // from own calculation 
// float alpha = 0.027947230169320696;  //50 Hz - 10 ksps in interrupt (from lecture)
// float alpha_new = 0.006243953391; // 10Hz
// float alpha_new = 0.05911739744;  // 100 Hz
// float alpha_new = 0.015465039;    // sampling rate 20000 Hz
// float oneMinusAlpha_new = 1.0-alpha_new; 
float oneMinusAlpha = 1.0-alpha; //Calculation to speed up filter calculation in ISR

volatile float oldY = 0.0;
volatile float newY = 0.0;

// Frequency Calculation
volatile long int zeroCrossing = 0;
volatile float sampleCount = 0.0;
float sample = 0.0;
volatile float frequency = 0.0;
volatile float debug =0.0;
const float samplingRate = 10000.0;

const float offsetValueZeroCrossing = 257.12;

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);
  analogWrite(7, 1);
  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(calculateFrequency);
  AdcBooster();
  DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
}

void loop() {
    digitalWrite(ledPin, 1);
    if (zeroCrossing % 20 == 0) {
      lcd.clear();
      lcd.print((frequency), 6);
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
  ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
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
  analogReadResolution(10);
  analogWriteResolution(10);
  sample = (float)analogRead(A1);
  newY = lowPassFilter(sample, oldY);

  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing and oldY < offsetValueZeroCrossing) {
    detectZeroCrossing();
    zeroCrossing++;
    sampleCount = 0.0;
    }
    else {
  sampleCount++;
    }
  oldY = newY;
  analogWrite(A0, newY);
    
}

void detectZeroCrossing() {
  // debug = ((newY*(1.0/samplingRate))/(newY-oldY));
  debug = (offsetValueZeroCrossing - oldY) * ((1.0/samplingRate)/(newY-oldY));
   
  // debug = ((newY - offsetValueZeroCrossing) *((1.0/samplingRate)/(newY-oldY)));
   // float period = (sampleCount/samplingRate)-((newY*(1.0/samplingRate))/(newY-oldY));
   float period = (sampleCount/samplingRate)-debug;
//   Serial.println("Plain Period: ");
//   Serial.println((sampleCount/samplingRate));
//  Serial.println("Interpol: ");
//  Serial.println(((newY*(1/samplingRate))/(newY-oldY)), 5);
//  Serial.println("Period: ");
//  Serial.println(period);
//    Serial.println("Y new: ");
//  Serial.println(newY);
//    Serial.println("Y old: ");
//  Serial.println(oldY);
  frequency = ((float) 1.0/period);
  
//  Serial.println(zeroCrossing);
//  Serial.println(sampleCount);
//  Serial.println(samplingRate);
//  Serial.println(frequency);
  }


  
