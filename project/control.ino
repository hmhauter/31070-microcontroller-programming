// This functions handle the control part for frequency control 
#include "setUpPIN.h"

void calculate_RMS() {
    RMS_voltage = 0.997 * sqrt(voltSum/(float)outerSampleCounter);
}

void generatorLoadControl() {
    if(frequency >= 50.1) {
      digitalWrite(loadPIN, 1);
      digitalWrite(generatorPIN, 0);
    } 
    else if(frequency <= 49.9) {
      digitalWrite(loadPIN, 0);
      digitalWrite(generatorPIN, 1);
    } else {
      digitalWrite(loadPIN, 0);
      digitalWrite(generatorPIN, 0);
     
    }
}

float droopControl() {
  return (carPower + ((baseFrequency - frequency) / proportionalGain));
}

float calculateCurrent(float droopPower) {
  return (droopPower / 240.0);
}

float dutyCycle(float current) {
  float PWM_percent;
  if (current <= 6.0) {
    PWM_percent = 10.0;   
    }
  else if(current >= 6.0 && current < 52.0 ) {
    PWM_percent = (75.0/46.0)*(current - 6.0) + 10.0;
    }
  else if(current >= 52.0 && current <= 52.5) {
    PWM_percent = 85.0;
    } 
  else if(current > 52.5 && current < 80.0) {
    PWM_percent = 0.4*(current - 52.5) + 85.0;
    }
  else if(current >= 80.0) {
    PWM_percent = 96.0;
    }
  return PWM_percent;
}
