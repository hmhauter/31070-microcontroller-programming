#ifndef CONTROL_DOT_H   
#define CONTROL_DOT_H 

void calculate_RMS();
void generatorLoadControl();
float droopControl();
float calculateCurrent(float droopPower);
float dutyCycle(float current);

#endif /* CONTROL_DOT_H */