#ifndef ZEROCROSSING_DOT_H   
#define ZEROCROSSING_DOT_H 

float lowPassFilter(float Input, float yOld);
void detectZeroCrossing(void);
float interpolation();
float calculateAlpha(float crossOverFreq, float deltaT);


#endif /* ZEROCROSSING_DOT_H */