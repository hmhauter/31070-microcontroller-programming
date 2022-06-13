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
const float offsetValueZeroCrossing = 257.12;

const int resolution = 1023; 

// Low Pass Filter
const float crossOverFreq = 50.0;


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
volatile float sampleCounter = 0.0;
volatile float power = 0.0;

volatile float frequencySum = 0.0;

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  alpha = calculateAlpha(crossOverFreq, (float)(1.0/samplingRate));
  oneMinusAlpha = 1.0 - alpha;
  
  analogReadResolution(10);
  analogWriteResolution(10);
  
  pinMode(ledPin, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  analogWrite(7, 1);
  digitalWrite(ledPin, 0);

  MyTimer5.begin(samplingRate);
  MyTimer5.attachInterrupt(calculateFrequency);
  AdcBooster();
  // DAC->CTRLA.bit.ENABLE = 1; // Enable DAC
  MyTimer5.start();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
}

void loop() {
//  Serial.println(alpha, 6);
//  Serial.print("\t");
//  Serial.println("Frequency: ");
//  Serial.println(frequency, 6);
//  Serial.println("RMS Ampere: ");
//  Serial.println(RMS_ampere, 6);
//  Serial.println("RMS Voltage: ");

//  Serial.println(RMS_voltage, 6);
//  Serial.println("Power: ");
//  Serial.println(power, 6);

  if (zeroCrossing % 20 == 0) {
    lcd.clear();
    lcd.print((frequency),3);
    frequencySum = 0.0;
    
  }
  if (frequency <= 49.9 || frequency >= 50.2 ) {
    digitalWrite(6, 1);
  }
  else {
    digitalWrite(6,0);
  }
}

void AdcBooster() {
  ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | // Divide Clock by 16.
  ADC_CTRLB_RESSEL_10BIT; // Result on 10 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample
  ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;//0x00; // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization
} 

float lowPassFilter(float Input, float yOld){
  //Low-pass filter function
  return (float) alpha*Input+oneMinusAlpha*yOld;  
} 

void calculateFrequency(void) {
  // TIMESTAMP FOR DEBUGGING 
  // nt = newT = micros() / 1000000.0;
  // Interrupt Callback function 
  sample = (float)analogRead(A1);
  // apply low pass filter to smoothen data
  newY = lowPassFilter(sample, oldY);
  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing and oldY < offsetValueZeroCrossing) {
    // get current timestamp
    newT = micros() / 1000000.0; // millis()/1000.0;
    // interpolation
    debug = ((newY - offsetValueZeroCrossing) / (newY - oldY)) * (1/ samplingRate);
    float period = newT - oldT- debug;
    frequency = (1.0/period);

    frequencySum += frequency;
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
      voltage += powf(map(sample, 0, resolution, -240*sqrt(2), 240*sqrt(2)), 2);
      ampere += powf(map(sample, 0, resolution, -42*sqrt(2), 42*sqrt(2)), 2);
      sampleCounter++;
      }
  oldY = newY;
  // ot = nt;
  analogWrite(A0, newY);
}

//void detectZeroCrossing() {
//  float interpolationValue = ((newY - offsetValueZeroCrossing) / (newY - oldY)) * (1.0/samplingRate);
//  // set current timestamp
//  newT = millis()/1000.0;
//  // calculate the period as the difference between the last zero crossing and the new zero crossing
//  float period = newT - oldT; //- interpolationValue;
//  frequency = ((float) 1.0/period);
//  oldT = newT;
//  }

float calculateAlpha(float crossOverFreq, float deltaT) {
  float RC = 1 / (2 * PI * crossOverFreq);
  return deltaT / (RC + deltaT);
  }

  
