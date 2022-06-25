// This functions handle the control part for frequency control 
#include "setUpPIN.h"

void calculate_RMS() {
    // use linear scaling factor to tune RMS voltage
    RMS_voltage = 0.9965 * sqrt(voltSum/(float)outerSampleCounter);
}

void generatorLoadControl() {
    if(frequency >= 50.1) {
      // activate the LED that presents the load 
      // deactivate the other LED
      digitalWrite(loadPIN, 1);
      digitalWrite(generatorPIN, 0);
    } 
    else if(frequency <= 49.9) {
      // activate the LED that presents the generator 
      // deactivate the other LED
      digitalWrite(loadPIN, 0);
      digitalWrite(generatorPIN, 1);
    } else {
      // "normal" frequency interval is reached 
      // deactivate all two LEDs
      digitalWrite(loadPIN, 0);
      digitalWrite(generatorPIN, 0);
     
    }
}

float droopControl() {
  float _power;
  // if the frequency is not inbetween 49.9 Hz and 50.1 do not apply control
  // leave values constant
  if(frequency <= 49.9) {
    _power = 1440;
  } else if (frequency >= 50.1) {
    _power = 19200;
  } else {
    // apply droop control  
    _power =  (carPower + ((baseFrequency - frequency) / proportionalGain));
  }
  return _power;
}

float calculateCurrent(float droopPower) {
  // use RMS voltage to get from power to current
  return (droopPower / RMS_voltage);
}

float dutyCycle(float current) {
  // follow calculation of pwm from lecture slides
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
