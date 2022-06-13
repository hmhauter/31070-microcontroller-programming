#include "Timer5.h"
#include <LiquidCrystal.h>
#include <math.h>
#include <stdbool.h>

#define PI 3.1415926535897932384626433832795
LiquidCrystal lcd(12, 4, 5, 11, 3, 2);

// Global Variables

// Debugging 
const int ledPin = 1;

// Frequency Calculation
volatile long int zeroCrossing = 0;

float sample = 0.0;
volatile float frequency = 0.0;
volatile float debug = 0.0;
const int samplingRate = 10000;
const int offsetValueZeroCrossing = 257;

const int resolution = 1023; 

// Low Pass Filter
const float crossOverFreq = 150.0;


float alpha;
// float alpha = 0.03045902795;      // from own calculation 
// float alpha = 0.027947230169320696;  //50 Hz - 10 ksps in interrupt (from lecture)
// float alpha_new = 0.006243953391; // 10Hz
// float alpha_new = 0.05911739744;  // 100 Hz
// float alpha_new = 0.015465039;    // sampling rate 20000 Hz
// float oneMinusAlpha_new = 1.0-alpha_new; 
float oneMinusAlpha; //Calculation to speed up filter calculation in ISR

volatile float oldY = 0.0;
volatile float newY = 0.0;

volatile float newT = 0.0;
volatile float oldT = 0.0;

volatile float nt = 0.0;
volatile float ot = 0.0;

volatile float voltage = 0.0;
volatile float ampere = 0.0;
volatile float RMS_voltage = 0.0;
volatile float RMS_ampere = 0.0;
volatile float power = 0.0;

const float frequencyPeriod = 1.0;
unsigned long previousMillis = 0;  

volatile int sampleCounter = 0;

volatile int t1 = 0;
volatile int t2 = 0;

const int measureFrequency = 50; 

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  alpha = calculateAlpha(crossOverFreq, (1.0/samplingRate));
  oneMinusAlpha = 1.0 - alpha;
  
  analogReadResolution(10);
  analogWriteResolution(10);
  
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  analogWrite(7, 1);
  digitalWrite(ledPin, 0);

  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(detectZeroCrossing);
  AdcBooster();
  // DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
}

void loop() {
//  unsigned long currentMillis = millis();
//  if (currentMillis - previousMillis >= 1000) {
//    lcd.clear();
//    lcd.print(zeroCrossing);
//    previousMillis = currentMillis;
//    zeroCrossing = 0;
//    }
  if (zeroCrossing >= measureFrequency) {
    // this is important!
    // use scaling factors for frequency measurement (emperically determined)
    // don't use sample Rate - it is not accurate enough
    frequency = zeroCrossing / (sampleCounter * (1.0 / 10927.0)*1.007375); // 1.00731
    lcd.clear();
    lcd.print(frequency, 4);
    zeroCrossing = 0;
    sampleCounter = 0;
    }



//  if (sampleCounter == samplingRate) {
//    frequency = (zeroCrossing-1) / ((t2-t1) * (1.0/samplingRate));
//    
//    lcd.clear();
//    lcd.print(frequency);
//    //lcd.setCursor(0, 1);
//    // lcd.print(sampleCounter);
//    zeroCrossing = 0;
//    sampleCounter = 0;
//    t1 = 0;
//    
//  }
//   
//  if (frequency <= 49.9 || frequency >= 50.2 ) {
//    digitalWrite(6, 1);
//  }
//  else {
//    digitalWrite(6,0);
//  }
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

void detectZeroCrossing(void) {

  // Interrupt Callback function 
  sample = analogRead(A1);
  
  // apply low pass filter to smoothen data
  newY = lowPassFilter(sample, oldY);
  sampleCounter++;
  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing && oldY < offsetValueZeroCrossing) {
    zeroCrossing++;
//    if (t1 == 0) {
//      t1 = sampleCounter;
//      }
//    t2 = sampleCounter;
  }  
  oldY = newY;

 analogWrite(A0, newY);
}

void calculateFrequency() {
  // float interpolationValue = ((newY - offsetValueZeroCrossing) / (newY - oldY)) * (1.0/samplingRate);
  // calculate the period as the difference between the last zero crossing and the new zero crossing
  
  // float period = newT - oldT; //- interpolationValue;
  frequency = (zeroCrossing-1) / ((t2-t1) * (1.0/samplingRate));
  }

float calculateAlpha(float crossOverFreq, float deltaT) {
  float RC = 1 / (2 * PI * crossOverFreq);
  return deltaT / (RC + deltaT);
  }

  
