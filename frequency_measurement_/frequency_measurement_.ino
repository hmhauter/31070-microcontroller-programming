#include "Timer5.h"

// Global Variables

// Debugging 
const int ledPin = 1;

// Low Pass Filter
float alpha = 0.027947230169320696; //50 Hz - 10 ksps in interrupt
float oneMinusAlpha = 1-alpha; //Calculation to speed up filter calculation in ISR

volatile float oldY = 0.0;
volatile float newY = 0.0;

// Frequency Calculation
volatile int zeroCrossing = 0;
volatile int freqCount = 0.0;
float sample = 0.0;


void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  MyTimer5.begin(10000);
  MyTimer5.attachInterrupt(calculateFrequency);
  AdcBooster();
  DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
}

void loop() {
    digitalWrite(ledPin, 1);
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
  float yNew;
  yNew = alpha*Input+oneMinusAlpha*yOld; 
  return yNew; 
} 

void calculateFrequency(void) {
  // Interrupt start

  // Calculate Frequency
  analogReadResolution(10);
  sample = analogRead(A1);
  newY = lowPassFilter(sample, oldY);
  oldY = newY;

  // detect "zero crossing"
  if(newY >= 176.75 and oldY < 176.75) {
    zeroCrossing++;
    }
  freqCount++;
  digitalWrite(A0, newY);
    
}
