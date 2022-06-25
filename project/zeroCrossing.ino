// This file holds all the functions to prepare the frequency calculation 
#include "setUpPIN.h"

float lowPassFilter(float Input, float yOld){
  //Low-pass filter function
  return (float) alpha*Input+oneMinusAlpha*yOld;  
} 

void detectZeroCrossing(void) {
  // Interrupt Callback function 
  sample = analogRead(readFrequencyPIN);
  // apply low pass filter to remove noisy data
  newY = lowPassFilter(sample, oldY);

  sampleCounter++;

  // detect "zero crossing"
  if(newY >= offsetValueZeroCrossing && oldY < offsetValueZeroCrossing) {
    zeroCrossing++;
    newY_int = newY;
    oldY_int = oldY;
  }  
  oldY = newY;  // update value for next interrupt

  // for debugging purpose (measure filtered signal)
  // comment this for the final version 
  // analogWrite(writeFrequencyPIN, newY);
}


float calculateAlpha(float crossOverFreq, float deltaT) {
  // calculate the constant for the low pass filter 
  float RC = 1 / (2 * PI * crossOverFreq);
  return deltaT / (RC + deltaT);
}

float interpolation() {
  // improve the accuracy of frequency calculation 
  // interpolate between two samples where zero crossing did occure
  float interpolX = ((float)newY_int-(float)offsetValueZeroCrossing) / ((float)newY_int-(float)oldY_int);
  return interpolX*(1.0 / 10927.0);
}

